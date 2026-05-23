"""Unit tests for VFH+ on small synthetic scenes."""

from __future__ import annotations

import pytest
from obstacle_avoidance.vfh.vfh_plus import VFHPlus


def _free_scan(n: int = 360, r: float = 8.0) -> tuple[list[float], list[float]]:
    return [i * (360.0 / n) for i in range(n)], [r] * n


def test_constraint_mu1_gt_mu2_plus_mu3():
    with pytest.raises(ValueError):
        VFHPlus(mu1=2.0, mu2=2.0, mu3=2.0)


def test_clear_path_returns_target_heading():
    angles, ranges = _free_scan()
    vfh = VFHPlus()
    theta = vfh.step(angles, ranges, target_heading_deg=0.0, current_heading_deg=0.0)
    assert theta is not None
    # With nothing in the way the planner should aim straight at the goal,
    # within one sector.
    assert min(abs(theta - 0.0), 360.0 - abs(theta - 0.0)) <= vfh.alpha


def test_obstacle_ahead_deflects_steering():
    angles, ranges = _free_scan()
    # Slam a wall of close hits in front (-20..20 deg, 1.0 m).
    for i, a in enumerate(angles):
        if a <= 20.0 or a >= 340.0:
            ranges[i] = 1.0
    vfh = VFHPlus()
    theta = vfh.step(angles, ranges, target_heading_deg=0.0, current_heading_deg=0.0)
    assert theta is not None
    # The chosen heading must NOT lie inside the blocked cone.
    in_blocked = theta <= 25.0 or theta >= 335.0
    assert not in_blocked, f"chose heading {theta} inside the blocked cone"


def test_dead_end_returns_none():
    angles, ranges = _free_scan(r=0.5)  # ring of close obstacles everywhere
    vfh = VFHPlus()
    theta = vfh.step(angles, ranges, target_heading_deg=0.0, current_heading_deg=0.0)
    assert theta is None


def test_step_is_deterministic_for_repeated_calls():
    angles, ranges = _free_scan()
    ranges[0:10] = [1.0] * 10
    vfh = VFHPlus()
    out = [vfh.step(angles, ranges, 0.0, 0.0) for _ in range(3)]
    assert out[0] == out[1] == out[2]
