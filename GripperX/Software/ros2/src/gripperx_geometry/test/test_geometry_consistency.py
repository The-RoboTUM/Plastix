"""Prove that every site holding a geometric value agrees with the declaration.

WHAT THIS TEST IS FOR
    ``config/geometry.yaml`` declares each geometric quantity once. This test
    reads back all 42 places those quantities are actually written down and
    fails when any of them has drifted.

WHY IT PASSES TODAY DESPITE 20 DISAGREEMENTS
    Twenty sites disagree with the declaration right now, on 2026-08-21, before
    anything has been changed. That is the problem this whole exercise exists to
    fix, and Phase 1 fixes it. Until then each one is registered below with what
    it holds and why — so the test is green on the KNOWN state and red on any
    NEW drift, instead of being red from birth and therefore ignored.

    The registry pins the VALUE, not just the site. A registered site that
    changes to some third value still fails. A registered site that gets fixed
    also fails, with an instruction to delete its entry — so the registry cannot
    quietly decay into a permanent amnesty for whatever happens to be on disk.
"""

from __future__ import annotations

import pytest

from gripperx_geometry.generate import constants_path, emit_python_constants
from gripperx_geometry.inventory import (
    Delegated,
    Imported,
    Missing,
    find_src_root,
    load_source,
    walk,
)


# ---------------------------------------------------------------------------
# The registry
# ---------------------------------------------------------------------------

STALE_CAD = (
    "Obsolete CAD pair a=0.203 / b=0.16556 (wheelbase 0.406, track 0.33112), "
    "superseded by the 2026-08-19 import. This site was never moved with the "
    "configs. A node started without its param file, or a test run at all, "
    "silently uses a robot that does not exist. Phase 1 removes it."
)

CORRECT_BUT_DUPLICATED = (
    "Holds 0.070, which is the declared value — and is registered anyway. A "
    "node default that happens to be right today is still a second source of "
    "truth: nothing keeps it in step, and it is precisely what the node falls "
    "back on when its parameter file is missing, i.e. when being right matters "
    "most. Phase 1 removes the default rather than correcting it."
)

SUPERSEDED_TAPE = (
    "Holds the tape pair a=0.180 / b=0.110 (measured 2026-08-13, commit "
    "275246c). GQ-1 was decided on 2026-08-21 in favour of the CAD figures as "
    "one quantity, so this is no longer an open question — it is a known-wrong "
    "value with a scheduled fix. Phase 1 corrects it.\n\n"
    "DEPLOY GATE, NOT A LANDING GATE: the correction moves the commanded "
    "in-place-spin steering angle 58.570 deg -> 58.999 deg. Phase 1 may land in "
    "the repo; the Pi must not receive it until a drive test has validated the "
    "change, and that test sits behind the OP-29 chirality test in the ordering "
    "constraint."
)

# check_teleop_manoeuvre_path.py moved from the obsolete pair to the config pair
# in fbd2276, between this registry being written and the branch being rebased.
# The test caught it unprompted, named both sites and said what had changed —
# the first live evidence that the mechanism works outside injected drift.

KNOWN_DIVERGENCES: dict[str, tuple[float, str]] = {
    # EMPTIED BY PHASE 1, 2026-08-21. Every site that held a geometry value now
    # either agrees with the declaration, delegates it to a parameter file, or
    # imports it. Nothing is registered because nothing diverges.
    #
    # An entry belongs here only while a KNOWN divergence is waiting on a
    # decision or a scheduled fix. It is not a place to park a failure.
}


@pytest.fixture(scope="module")
def sites():
    return walk()


@pytest.fixture(scope="module")
def source():
    return load_source()


# ---------------------------------------------------------------------------
# The declaration itself has to be sound before it can judge anything
# ---------------------------------------------------------------------------


def test_source_is_wellformed(source):
    allowed_provenance = {"cad", "measured", "derived", "rounded_from"}
    allowed_status = {"accepted", "TO-VERIFY"}
    for name, spec in source["quantities"].items():
        assert "value" in spec, f"{name}: no value"
        assert spec.get("unit"), f"{name}: no unit"
        assert spec.get("provenance") in allowed_provenance, f"{name}: bad provenance"
        assert spec.get("status") in allowed_status, f"{name}: bad status"
        assert spec.get("rationale", "").strip(), f"{name}: no rationale"
        assert isinstance(spec.get("decimals"), int), f"{name}: no decimals declared"
        if spec["provenance"] == "derived":
            assert spec.get("derived_from"), f"{name}: derived without derived_from"


def test_decided_quantities_record_their_decision(source):
    """A value chosen between competing determinations must say so, and why.

    Without this, the CAD figures would look like plain CAD provenance and the
    tape measurement that lost would vanish from the record — leaving the next
    reader to rediscover the disagreement, which is how it survived this long.
    """
    for name in ("half_wheelbase_kingpin", "half_track_kingpin"):
        decision = source["quantities"][name].get("decision")
        assert decision, f"{name}: chosen against a competing value, but no decision block"
        for field in ("question", "by", "date", "outcome"):
            assert decision.get(field), f"{name}: decision has no {field}"


def test_resolved_questions_carry_their_resolution(source):
    for key, entry in (source.get("open_questions") or {}).items():
        if "RESOLVED" in entry:
            assert entry.get("resolution", "").strip(), f"{key}: resolved without a resolution"


def test_generated_constants_are_current(source):
    """The one generated file that lives in the tree must not go stale.

    constants.py is committed rather than staged because tests import it, and a
    staging directory is not importable. That makes it the one place where a
    generated artefact can silently fall behind its source — so it is checked
    rather than trusted. Regenerate with:

        python3 -m gripperx_geometry.generate --sync-constants
    """
    on_disk = constants_path(find_src_root()).read_text()
    assert on_disk == emit_python_constants(source), (
        "constants.py is out of date with config/geometry.yaml; regenerate with "
        "`python3 -m gripperx_geometry.generate --sync-constants`"
    )


def test_every_consumer_site_still_exists(sites):
    """A locator that stops resolving is drift too — the same rot, one level up."""
    missing = [s for s in sites if isinstance(s.actual, Missing)]
    assert not missing, "consumer sites named in geometry.yaml no longer resolve:\n" + "\n".join(
        f"  - {s.quantity}: {s.describe()}" for s in missing
    )


# ---------------------------------------------------------------------------
# Consumers against the declaration
# ---------------------------------------------------------------------------


def test_node_defaults_carry_no_geometry_value(sites):
    """No node may hold a numeric default for a declared geometric quantity.

    This is the invariant Phase 1 establishes, asserted here from the start so
    the target state is testable before it is reached. A default in code is a
    second source of truth by construction: it is what the node uses when its
    parameter file is absent, and nothing keeps it in step. That is how three
    nodes came to fall back on a robot with a 0.406 m wheelbase.

    Every site still holding one is registered below; the registry empties as
    Phase 1 lands.
    """
    holding = [
        s for s in sites
        if s.locator.kind == "declare_parameter" and not isinstance(s.actual, Delegated)
    ]
    unregistered = [s for s in holding if s.locator.raw not in KNOWN_DIVERGENCES]
    assert not unregistered, "node defaults holding a geometry value, unregistered:\n" + "\n".join(
        f"  - {s.locator.raw}: {s.actual!r}" for s in unregistered
    )


def test_no_undeclared_divergence(sites):
    """Any site that disagrees must be registered, with the value it holds."""
    problems = []
    for site in sites:
        if site.agrees:
            continue
        if site.locator.kind == "declare_parameter":
            # Node defaults are judged by test_node_defaults_carry_no_geometry_value,
            # whose criterion is "holds nothing", not "holds the right number".
            continue
        entry = KNOWN_DIVERGENCES.get(site.locator.raw)
        if entry is None:
            problems.append(
                f"  NEW DRIFT in {site.quantity}:\n    {site.describe()}\n"
                f"      Fix the site, or edit config/geometry.yaml if the "
                f"declaration is what is wrong."
            )
            continue
        registered_value, _ = entry
        if not isinstance(site.actual, (int, float)) or abs(site.actual - registered_value) > 1e-12:
            problems.append(
                f"  REGISTERED SITE MOVED in {site.quantity}:\n    {site.describe()}\n"
                f"      The registry expected it to hold {registered_value!r}. "
                f"It now holds something else, which no known divergence covers."
            )
    assert not problems, "geometry drift:\n" + "\n".join(problems)


def test_registry_has_no_stale_entries(sites):
    """A fixed site must lose its entry, or the registry becomes an amnesty."""
    by_locator = {s.locator.raw: s for s in sites}
    stale = []
    for raw in KNOWN_DIVERGENCES:
        site = by_locator.get(raw)
        if site is None:
            stale.append(f"  - {raw}\n      no longer a declared consumer; drop the entry")
        elif site.agrees:
            if isinstance(site.actual, Imported):
                what = "now imports from the declaration"
            elif site.locator.kind == "declare_parameter":
                what = "now delegates to the parameter file"
            else:
                what = "now agrees with the declaration"
            stale.append(f"  - {raw}\n      {what}; drop the entry")
    assert not stale, "KNOWN_DIVERGENCES entries that have outlived their reason:\n" + "\n".join(stale)


# ---------------------------------------------------------------------------
# Internal consistency of the declaration — the couplings that have broken
# ---------------------------------------------------------------------------


def _value(source, name):
    return source["quantities"][name]["value"]


def test_contact_convention_is_kingpin_plus_lever_arm(source):
    """The 51 % gap between the two conventions must be exactly the lever arm.

    If this ever fails, the two conventions have stopped describing the same
    robot, and OP-29's arithmetic no longer holds.
    """
    expected = _value(source, "half_track_kingpin") + _value(source, "kingpin_to_contact_lateral")
    assert abs(_value(source, "half_track_contact") - expected) < 5e-5


def test_contact_half_spacings_are_the_means_they_claim_to_be(source):
    positions = _value(source, "wheel_positions_contact")
    xs = [abs(v) for v in positions[0::2]]
    ys = [abs(v) for v in positions[1::2]]
    assert abs(_value(source, "half_wheelbase_contact") - sum(xs) / len(xs)) < 5e-6
    assert abs(_value(source, "half_track_contact") - sum(ys) / len(ys)) < 5e-6


def test_full_spans_are_twice_the_halves(source):
    assert abs(_value(source, "track_width") - 2 * _value(source, "half_track_kingpin")) < 1e-9
    assert abs(_value(source, "wheelbase") - 2 * _value(source, "half_wheelbase_kingpin")) < 1e-9


def test_ride_height_still_tracks_the_wheel_radius(source):
    """The coupling that put the wheels 4.2 mm underground once already.

    base_footprint_to_base_link_z is not independent: it is the chassis
    underside plus the geometric wheel radius. Changing the radius without
    changing this is always a bug, so it is asserted rather than trusted.
    """
    chassis_underside = 0.0866
    expected = chassis_underside + _value(source, "wheel_radius_geometric")
    assert abs(_value(source, "base_footprint_to_base_link_z") - expected) < 1e-9, (
        "ride height and wheel radius have come apart — see the coupled_to note "
        "on base_footprint_to_base_link_z in config/geometry.yaml"
    )


def test_the_two_radii_are_not_silently_merged(source):
    """Guard the distinction, not the equality.

    wheel_radius_geometric and wheel_radius_effective are equal today by
    coincidence of measurement. This test does not assert that they stay equal —
    it asserts they stay two entries, so that a future outdoor measurement can
    move one without dragging the other with it.
    """
    quantities = source["quantities"]
    assert "wheel_radius_geometric" in quantities
    assert "wheel_radius_effective" in quantities
    assert quantities["wheel_radius_geometric"]["provenance"] == "cad"
    assert quantities["wheel_radius_effective"]["provenance"] == "measured"
