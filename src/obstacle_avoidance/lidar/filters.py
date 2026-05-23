"""Post-processing filters applied between the LD19 driver and VFH+."""

from __future__ import annotations

import numpy as np

from ..config import LidarConfig
from .ld19 import Scan


def _angular_distance(a: float, b: float) -> float:
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def median_filter_circular(values: list[float], window: int) -> list[float]:
    """Wrap-aware odd-window median filter on a circular angular signal."""
    if window <= 1:
        return list(values)
    if window % 2 == 0:
        raise ValueError("median window must be odd")
    n = len(values)
    if n == 0:
        return []
    half = window // 2
    arr = np.asarray(values, dtype=float)
    # Pad by wrapping the start/end together.
    padded = np.concatenate([arr[-half:], arr, arr[:half]])
    out = np.empty(n, dtype=float)
    for i in range(n):
        out[i] = np.median(padded[i : i + window])
    return out.tolist()


def apply_filters(scan: Scan, cfg: LidarConfig) -> Scan:
    """Return a new Scan with intensity / range / mask / median filters applied."""
    angles: list[float] = []
    ranges: list[float] = []
    intens: list[int] = []

    mask = cfg.prop_mask_deg
    for a, r, i in zip(scan.angles_deg, scan.ranges_m, scan.intensities, strict=False):
        if i < cfg.min_intensity:
            continue
        if r < cfg.range_min_m:
            continue
        r_clipped = min(r, cfg.range_max_m)
        if mask is not None:
            lo, hi = mask
            in_mask = lo <= a <= hi if lo <= hi else (a >= lo or a <= hi)
            if in_mask:
                r_clipped = cfg.range_max_m  # treat as free space
        angles.append(a)
        ranges.append(r_clipped)
        intens.append(i)

    if cfg.median_window > 1 and len(ranges) >= cfg.median_window:
        ranges = median_filter_circular(ranges, cfg.median_window)

    return Scan(angles_deg=angles, ranges_m=ranges, intensities=intens)
