"""Emit the derived artefacts from config/geometry.yaml.

PHASE 0 SCOPE — READ THIS BEFORE WIRING ANYTHING UP
    Nothing this module writes is consumed by the build yet. It writes into a
    staging directory so the mechanism can be reviewed and proven against the
    live tree without touching a single file another session depends on.

    NOT YET EMITTED: the four per-corner ``*_wheel_offset_xyz`` triples in
    core.xacro. They combine a per-corner longitudinal asymmetry, the declared
    lateral lever arm and a vertical drop, and only the middle one is a declared
    quantity today. Phase 1 has to model the other two before that consumer can
    be generated rather than merely checked.

    Phase 1 is what points the consumers at these outputs. That step is
    deliberately not taken here, and one of the outputs cannot be taken at all
    until GQ-1 is answered: the control configs hold a=0.180 / b=0.110 against
    the declared CAD 0.1809 / 0.1087, and generating them today would silently
    impose one reading of that question on the robot.

USAGE
    python3 -m gripperx_geometry.generate --out /tmp/geometry-staging
    python3 -m gripperx_geometry.generate --check
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

from gripperx_geometry.inventory import find_src_root, load_source, walk

BANNER = "GENERATED FROM gripperx_geometry/config/geometry.yaml — DO NOT EDIT"


def fmt(spec: dict[str, Any], value: float) -> str:
    """Render a number with the decimal places the declaration asks for.

    Not cosmetic. Python renders 0.070 as ``0.07``, so generating a file that
    currently reads ``0.070`` would churn every diff with changes that are not
    changes — and a Phase 1 review whose whole claim is "no value moved" cannot
    afford noise that looks like movement. The place count is declared per
    quantity rather than inferred, so it stays a decision and not a guess.
    """
    return f"{value:.{spec['decimals']}f}"


def _header(comment: str) -> str:
    return (
        f"{comment} {BANNER}\n"
        f"{comment} Edit the quantity there and regenerate; edits here are lost.\n"
    )


# ---------------------------------------------------------------------------
# Emitters
# ---------------------------------------------------------------------------


def emit_python_constants(source: dict[str, Any]) -> str:
    """The module the test files import instead of carrying literals."""
    lines = [
        '"""Geometric constants for GripperX.',
        "",
        BANNER,
        "",
        "Import these rather than writing a number into a test. A literal in a",
        "test file is a second source of truth that nothing keeps in step, which",
        "is how three test files came to validate against a robot that had not",
        "existed since 2026-08-19.",
        '"""',
        "",
    ]
    for name, spec in source["quantities"].items():
        value = spec["value"]
        lines.append(f"# {spec['provenance']}, {spec['status']}, {spec.get('date', 'n/a')}")
        if isinstance(value, list):
            lines.append(f"{name.upper()} = (")
            for item in value:
                lines.append(f"    {fmt(spec, item)},")
            lines.append(")")
        else:
            lines.append(f"{name.upper()} = {fmt(spec, value)}")
        lines.append("")
    return "\n".join(lines)


def emit_xacro_fragment(source: dict[str, Any]) -> str:
    """The property block core.xacro includes instead of declaring inline."""
    out = ['<?xml version="1.0"?>', f"<!-- {BANNER} -->", "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"]
    for name, spec in source["quantities"].items():
        value = spec["value"]
        if isinstance(value, list):
            continue
        out.append(f'  <xacro:property name="{name}" value="{fmt(spec, value)}" />')
    out.append("</robot>")
    return "\n".join(out) + "\n"


def emit_param_fragments(source: dict[str, Any]) -> dict[str, str]:
    """One ROS param fragment per consuming config file.

    Keyed by the config's relative path, so Phase 1 can splice each one into the
    file it belongs to rather than inventing a new load order.
    """
    by_file: dict[str, dict[str, dict[str, Any]]] = {}
    for name, spec in source["quantities"].items():
        for raw in spec.get("consumers") or []:
            relpath, _, selector = raw.partition("::")
            if "." not in selector or "[" in selector:
                continue
            node, _, param = selector.partition(".")
            by_file.setdefault(relpath, {}).setdefault(node, {})[param] = spec

    rendered = {}
    for relpath, nodes in sorted(by_file.items()):
        lines = [_header("#").rstrip(), ""]
        for node, params in sorted(nodes.items()):
            lines.append(f"{node}:")
            lines.append("  ros__parameters:")
            for param, spec in sorted(params.items()):
                value = spec["value"]
                if isinstance(value, list):
                    lines.append(f"    {param}:")
                    lines.extend(f"      - {fmt(spec, item)}" for item in value)
                else:
                    lines.append(f"    {param}: {fmt(spec, value)}")
            lines.append("")
        rendered[relpath] = "\n".join(lines)
    return rendered


def emit_markdown_table(source: dict[str, Any]) -> str:
    """The documentation table, so prose can reference names instead of numbers."""
    lines = [
        f"<!-- {BANNER} -->",
        "",
        "# GripperX robot geometry",
        "",
        "| Quantity | Value | Unit | Provenance | Status | Sites |",
        "|---|---|---|---|---|---|",
    ]
    for name, spec in source["quantities"].items():
        value = spec["value"]
        shown = "see source" if isinstance(value, list) else f"`{fmt(spec, value)}`"
        lines.append(
            f"| `{name}` | {shown} | {spec['unit']} | {spec['provenance']} "
            f"| {spec['status']} | {len(spec.get('consumers') or [])} |"
        )
    questions = source.get("open_questions") or {}
    if questions:
        lines += ["", "## Open questions", ""]
        for key, entry in questions.items():
            lines.append(f"- **{key}** — {entry['title']} (owner: {entry['owner']})")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------


def constants_path(src_root: Path) -> Path:
    """Where the checked-in generated constants module lives."""
    return src_root / "gripperx_geometry" / "src" / "gripperx_geometry" / "constants.py"


def sync_constants(src_root: Path | None = None) -> Path:
    """Rewrite the checked-in constants module from the declaration.

    This one artefact is committed rather than staged, because tests have to
    import it and an uninstalled staging directory is not importable. A
    generated file in the tree needs a guard against going stale, which is
    test_generated_constants_are_current.
    """
    root = src_root or find_src_root()
    target = constants_path(root)
    target.write_text(emit_python_constants(load_source(root)))
    return target


def write_all(out_dir: Path, source: dict[str, Any]) -> list[Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []

    target = out_dir / "geometry_constants.py"
    target.write_text(emit_python_constants(source))
    written.append(target)

    target = out_dir / "geometry.xacro"
    target.write_text(emit_xacro_fragment(source))
    written.append(target)

    target = out_dir / "GEOMETRY.md"
    target.write_text(emit_markdown_table(source))
    written.append(target)

    fragments = out_dir / "param_fragments"
    fragments.mkdir(exist_ok=True)
    for relpath, text in emit_param_fragments(source).items():
        target = fragments / relpath.replace("/", "__")
        target.write_text(text)
        written.append(target)

    return written


def check() -> int:
    """Report how the live tree stands against the declaration. Never writes."""
    sites = walk()
    diverging = [s for s in sites if not s.agrees]
    print(f"{len(sites)} consumer sites declared")
    print(f"{len(sites) - len(diverging)} agree, {len(diverging)} diverge")
    if diverging:
        print("\ndiverging:")
        for site in diverging:
            print(f"  [{site.quantity}] {site.describe()}")
    return 1 if diverging else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, help="staging directory to write into")
    parser.add_argument("--check", action="store_true", help="report drift, write nothing")
    parser.add_argument(
        "--sync-constants",
        action="store_true",
        help="rewrite the checked-in constants module from the declaration",
    )
    args = parser.parse_args(argv)

    if args.check:
        return check()
    if args.sync_constants:
        print(f"wrote {sync_constants()}")
        return 0
    if not args.out:
        parser.error("give --out DIR or --check")

    source = load_source(find_src_root())
    for path in write_all(args.out, source):
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
