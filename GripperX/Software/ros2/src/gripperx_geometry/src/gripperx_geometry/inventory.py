"""Read the declared geometry, and read back what every consumer actually holds.

This module is the shared engine behind two things:

  * ``test/test_geometry_consistency.py`` — proves the consumers agree with
    ``config/geometry.yaml``, and fails when they drift.
  * ``generate.py`` — writes derived artefacts from the same declaration.

They share the extraction layer on purpose. A generator that emits values by one
route and a test that reads them back by another can agree with each other while
both being wrong about the file on disk.

Nothing here decides anything. Where a value is ambiguous, the caller is told
what is on disk and what was declared, and the difference is surfaced — never
rounded away, never assumed to be a typo.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

CORNERS = ("front_left", "back_left", "back_right", "front_right")


# --------------------------------------------------------------------------
# Locating things
# --------------------------------------------------------------------------


def find_src_root(start: Path | None = None) -> Path:
    """Walk up until we find the workspace's ROS 2 ``src`` directory.

    Anchored on a file that has to exist for any of this to be meaningful,
    rather than on a directory name, which could match anywhere.
    """
    here = (start or Path(__file__)).resolve()
    for candidate in [here, *here.parents]:
        if (candidate / "gripperx_control" / "config" / "swerve_cmd.yaml").is_file():
            return candidate
    raise RuntimeError(
        f"could not locate the ROS 2 src root by walking up from {here}; "
        "expected an ancestor containing gripperx_control/config/swerve_cmd.yaml"
    )


def source_path(src_root: Path) -> Path:
    return src_root / "gripperx_geometry" / "config" / "geometry.yaml"


def load_source(src_root: Path | None = None) -> dict[str, Any]:
    root = src_root or find_src_root()
    with source_path(root).open() as handle:
        return yaml.safe_load(handle)


# --------------------------------------------------------------------------
# Consumer locators
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Locator:
    """One place a quantity is written down, parsed from its ``consumers`` entry."""

    raw: str
    relpath: str
    selector: str

    @property
    def kind(self) -> str:
        if self.selector.startswith("cpp_member["):
            return "cpp_member"
        if self.selector.startswith("cpp_constexpr["):
            return "cpp_constexpr"
        if self.selector.startswith("cpp_array["):
            return "cpp_array"
        if self.selector.startswith("cpp_ctor["):
            return "cpp_ctor"
        if self.selector.startswith("declare_parameter_list["):
            return "declare_parameter_list"
        if self.selector.startswith("xacro:property["):
            return "xacro_property"
        if self.selector.startswith("declare_parameter["):
            return "declare_parameter"
        if self.selector.startswith("MODEL["):
            return "model_kwarg"
        if "." in self.selector:
            return "ros_param"
        return "py_constant"

    @property
    def argument(self) -> str:
        """The name inside the brackets, for the bracketed selector kinds."""
        match = re.fullmatch(r"[A-Za-z_:]+\[(.+)\]", self.selector)
        if not match:
            raise ValueError(f"selector {self.selector!r} carries no bracketed name")
        return match.group(1)


def parse_locator(raw: str) -> Locator:
    if "::" not in raw:
        raise ValueError(f"malformed consumer locator (no '::'): {raw!r}")
    relpath, selector = raw.split("::", 1)
    return Locator(raw=raw, relpath=relpath.strip(), selector=selector.strip())


# --------------------------------------------------------------------------
# Extraction — one handler per locator kind
# --------------------------------------------------------------------------


class Missing:
    """Sentinel: the site named by a locator does not exist.

    Distinct from a wrong value, and reported differently. A locator that no
    longer resolves usually means the file was refactored and this declaration
    was not updated with it — which is the same class of rot the whole exercise
    is about, so it must never be silently skipped.
    """

    def __init__(self, why: str) -> None:
        self.why = why

    def __repr__(self) -> str:  # pragma: no cover - diagnostic only
        return f"<missing: {self.why}>"


class Delegated:
    """Sentinel: the site exists and deliberately holds no value.

    A node that declares a parameter WITHOUT a default cannot start unless its
    parameter file supplies one — which is the whole point of Phase 1. Such a
    site is not missing and not divergent; it has handed the value to the
    declaration, which is the correct end state. Modelling it as `Missing`
    would make the target state look like rot.
    """

    def __repr__(self) -> str:  # pragma: no cover - diagnostic only
        return "<delegated to the parameter file>"


class Imported:
    """Sentinel: the site takes its value from the declaration by import.

    A test that reads `HALF_WHEELBASE_KINGPIN as A` no longer holds a literal
    to compare, and that is the point — there is nothing left to drift. Like
    `Delegated`, this is a target state, not a missing site.
    """

    def __repr__(self) -> str:  # pragma: no cover - diagnostic only
        return "<imported from gripperx_geometry.constants>"


CONSTANTS_IMPORT = "from gripperx_geometry.constants import"


def _imports_constants(text: str, local_name: str) -> bool:
    """True when the file pulls this local name out of the generated module."""
    if CONSTANTS_IMPORT not in text:
        return False
    return bool(
        re.search(rf"\bas\s+{re.escape(local_name)}\b", text)
        or re.search(rf"^\s*{re.escape(local_name)}\s*,\s*$", text, re.M)
    )


_NUMBER = r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"


def _read(src_root: Path, relpath: str) -> str | Missing:
    path = src_root / relpath
    if not path.is_file():
        return Missing(f"file not found: {relpath}")
    return path.read_text()


def _extract_ros_param(text: str, selector: str) -> Any:
    node, _, param = selector.partition(".")
    document = yaml.safe_load(text)
    if node not in document:
        return Missing(f"no top-level node key {node!r}")
    params = (document[node] or {}).get("ros__parameters", {})
    if param not in params:
        return Missing(f"{node} declares no parameter {param!r}")
    return params[param]


def _extract_xacro_property(text: str, name: str) -> Any:
    match = re.search(
        rf'<xacro:property\s+name="{re.escape(name)}"\s+value="([^"]*)"',
        text,
    )
    if not match:
        return Missing(f"no xacro:property named {name!r}")
    literal = match.group(1).strip()
    if re.fullmatch(_NUMBER, literal):
        return float(literal)
    if re.fullmatch(rf"(?:{_NUMBER}\s+){{2}}{_NUMBER}", literal):
        return [float(part) for part in literal.split()]
    return Missing(f"{name} is an expression, not a literal: {literal!r}")


def _extract_declare_parameter(text: str, name: str) -> Any:
    match = re.search(
        rf"declare_parameter\(\s*['\"]{re.escape(name)}['\"]\s*,\s*({_NUMBER})",
        text,
    )
    if not match:
        if re.search(rf"declare_parameter\(\s*['\"]{re.escape(name)}['\"]", text):
            return Delegated()
        return Missing(f"no declare_parameter for {name!r}")
    return float(match.group(1))


def _extract_model_kwarg(text: str, name: str) -> Any:
    call = re.search(r"FourWIS4WIDKinematicModel\(([^)]*)\)", text, re.S)
    if not call:
        return Missing("no FourWIS4WIDKinematicModel(...) call")
    match = re.search(rf"\b{re.escape(name)}\s*=\s*({_NUMBER})", call.group(1))
    if match:
        return float(match.group(1))
    bound = re.search(rf"\b{re.escape(name)}\s*=\s*([A-Z_][A-Z0-9_]*)", call.group(1))
    if bound and _imports_constants(text, bound.group(1)):
        return Imported()
    return Missing(f"model call passes no {name}=")


def _extract_py_constant(text: str, name: str) -> Any:
    match = re.search(rf"^{re.escape(name)}\s*=\s*({_NUMBER})\s*$", text, re.M)
    if match:
        return float(match.group(1))
    if _imports_constants(text, name):
        return Imported()
    return Missing(f"no module-level constant {name}")


def _extract_cpp_member(text: str, name: str) -> Any:
    """`double a_{0.180};` — an auto_declare fallback, i.e. what the C++
    controller uses when its YAML does not carry the key. Same class of second
    source of truth as a Python declare_parameter default."""
    match = re.search(rf"\b{re.escape(name)}\s*\{{\s*({_NUMBER})\s*\}}", text)
    if not match:
        return Missing(f"no member initialiser for {name}")
    return float(match.group(1))


def _extract_cpp_constexpr(text: str, name: str) -> Any:
    match = re.search(rf"constexpr\s+double\s+{re.escape(name)}\s*=\s*({_NUMBER})", text)
    if not match:
        return Missing(f"no constexpr double {name}")
    return float(match.group(1))


def _extract_cpp_array(text: str, name: str) -> Any:
    """`constexpr std::array<...> kName = {a, b, c, d};` — order matters."""
    match = re.search(rf"\b{re.escape(name)}\s*=\s*\{{(.*?)\}}", text, re.S)
    if not match:
        return Missing(f"no array initialiser for {name}")
    return [float(v) for v in re.findall(_NUMBER, match.group(1))]


def _extract_cpp_ctor(text: str, spec: str) -> Any:
    """`cpp_ctor[SwerveKinematics:0]` — the nth positional numeric argument.

    Every occurrence must agree; a file that constructs the model twice with
    two different geometries is itself the defect, so disagreement is reported
    rather than resolved to the first hit.
    """
    type_name, _, index = spec.partition(":")
    found = set()
    for call in re.finditer(rf"\b{re.escape(type_name)}\s*\w*\s*\(([^)]*)\)", text):
        numbers = re.findall(_NUMBER, call.group(1))
        if len(numbers) > int(index):
            found.add(float(numbers[int(index)]))
    if not found:
        return Missing(f"no {type_name}(...) call with a positional argument {index}")
    if len(found) > 1:
        return Missing(f"{type_name}(...) constructed with differing values: {sorted(found)}")
    return found.pop()


def _extract_declare_parameter_list(text: str, name: str) -> Any:
    match = re.search(
        rf"declare_parameter\(\s*['\"]{re.escape(name)}['\"]\s*,\s*\[(.*?)\]",
        text, re.S)
    if not match:
        if re.search(rf"declare_parameter\(\s*['\"]{re.escape(name)}['\"]", text):
            return Delegated()
        return Missing(f"no declare_parameter list for {name!r}")
    return [float(v) for v in re.findall(_NUMBER, match.group(1))]


def extract(src_root: Path, locator: Locator) -> Any:
    """Return what the consumer site actually holds, or a ``Missing``."""
    text = _read(src_root, locator.relpath)
    if isinstance(text, Missing):
        return text

    if locator.kind == "cpp_member":
        return _extract_cpp_member(text, locator.argument)
    if locator.kind == "cpp_constexpr":
        return _extract_cpp_constexpr(text, locator.argument)
    if locator.kind == "cpp_array":
        return _extract_cpp_array(text, locator.argument)
    if locator.kind == "cpp_ctor":
        return _extract_cpp_ctor(text, locator.argument)
    if locator.kind == "declare_parameter_list":
        return _extract_declare_parameter_list(text, locator.argument)
    if locator.kind == "ros_param":
        return _extract_ros_param(text, locator.selector)
    if locator.kind == "xacro_property":
        return _extract_xacro_property(text, locator.argument)
    if locator.kind == "declare_parameter":
        return _extract_declare_parameter(text, locator.argument)
    if locator.kind == "model_kwarg":
        return _extract_model_kwarg(text, locator.argument)
    if locator.kind == "py_constant":
        return _extract_py_constant(text, locator.selector)
    raise AssertionError(f"unhandled locator kind {locator.kind!r}")


# --------------------------------------------------------------------------
# Expected values
# --------------------------------------------------------------------------


def expected_for(quantity: str, spec: dict[str, Any], locator: Locator) -> Any:
    """What this particular site should hold, given the declaration.

    Usually just the quantity's value. The exception is a per-corner quantity,
    where the xacro carries an ``x y z`` triple and only the y component is the
    declared lateral offset — so the corner is read out of the property name.
    """
    per_corner = spec.get("per_corner")
    if per_corner and locator.kind == "xacro_property":
        name = locator.argument
        for corner in CORNERS:
            if name.startswith(corner):
                return per_corner[corner]
        return Missing(f"cannot tell which corner {name!r} belongs to")
    return spec["value"]


def project(spec: dict[str, Any], locator: Locator, actual: Any) -> Any:
    """Narrow a consumer's raw reading to the part the quantity actually claims.

    A per-corner lateral offset lives inside an ``x y z`` triple in the xacro.
    Only y is the declared quantity; x is the per-corner longitudinal asymmetry
    and z is the vertical drop, both of which are their own concerns. Comparing
    the whole triple against a scalar would report a divergence that is not one.
    """
    if spec.get("per_corner") and locator.kind == "xacro_property":
        if isinstance(actual, list) and len(actual) == 3:
            return actual[1]
    return actual


def compare(expected: Any, actual: Any, *, tolerance: float = 0.0) -> bool:
    """Exact by default. Tolerance is opt-in, per site, and never a default.

    A tolerance silently applied is how a real discrepancy gets absorbed. GQ-1
    is exactly such a case and is carried as a known divergence instead.
    """
    if isinstance(actual, Missing) or isinstance(expected, Missing):
        return False
    if isinstance(expected, list) != isinstance(actual, list):
        return False
    if isinstance(expected, list):
        if len(expected) != len(actual):
            return False
        return all(abs(e - a) <= tolerance for e, a in zip(expected, actual))
    if isinstance(actual, (int, float)) and isinstance(expected, (int, float)):
        # A per-corner offset is signed by side; the xacro triple carries the
        # sign and so does the declaration, so this stays a signed comparison.
        return abs(expected - actual) <= tolerance
    return expected == actual


@dataclass(frozen=True)
class Site:
    """One resolved consumer site with its verdict."""

    quantity: str
    locator: Locator
    expected: Any
    actual: Any
    agrees: bool

    def describe(self) -> str:
        if isinstance(self.actual, Imported):
            return f"{self.locator.raw}\n      imported from gripperx_geometry.constants"
        if isinstance(self.actual, Delegated):
            return f"{self.locator.raw}\n      delegated to the parameter file (no default)"
        if isinstance(self.actual, Missing):
            return f"{self.locator.raw}\n      site not found — {self.actual.why}"
        return (
            f"{self.locator.raw}\n"
            f"      declared {self.expected!r}, on disk {self.actual!r}"
        )


def walk(src_root: Path | None = None, source_root: Path | None = None) -> list[Site]:
    """Resolve every consumer of every quantity and report agreement.

    ``src_root`` is the tree the consumers are read from; ``source_root`` is the
    tree the declaration is read from, and defaults to the same one.

    They are separable on purpose. Checking a branch's declaration against a
    tree that has moved on underneath it is exactly the question "is this still
    mergeable" — and that came up for real on 2026-08-21, when Theo advanced 44
    commits mid-work and one registered site changed value underneath the
    branch. With the two roots welded together that check cannot be expressed.
    """
    root = src_root or find_src_root()
    source = load_source(source_root or root)
    sites: list[Site] = []
    for quantity, spec in source["quantities"].items():
        for raw in spec.get("consumers") or []:
            locator = parse_locator(raw)
            expected = expected_for(quantity, spec, locator)
            actual = project(spec, locator, extract(root, locator))
            if isinstance(actual, Imported):
                agrees = True
            elif locator.kind in ("declare_parameter", "declare_parameter_list"):
                # For a node default the target is not "holds the right value"
                # but "holds no value at all". A numeric default there is a
                # second source of truth by construction, however current it
                # happens to be today.
                agrees = isinstance(actual, Delegated)
            else:
                agrees = compare(expected, actual)
            sites.append(
                Site(quantity=quantity, locator=locator, expected=expected,
                     actual=actual, agrees=agrees)
            )
    return sites
