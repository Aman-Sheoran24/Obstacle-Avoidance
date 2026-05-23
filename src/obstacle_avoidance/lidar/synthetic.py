"""A drop-in fake LiDAR for SITL / offline development.

Implements the same ``read_scan()`` / ``close()`` contract as ``LD19`` so the
rest of the pipeline doesn't know (or care) which sensor it's running on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .ld19 import Scan


@dataclass
class _Box:
    x: float
    y: float
    w: float  # full width along X
    h: float  # full height along Y


@dataclass
class _Cylinder:
    x: float
    y: float
    r: float


class SyntheticLidar:
    """Ray-cast a static 2D scene at 1-degree resolution.

    Coordinates: sensor at origin, 0 deg = +X, 90 deg = +Y (CCW).
    All distances are in metres.
    """

    def __init__(
        self,
        range_min: float = 0.05,
        range_max: float = 12.0,
        n_beams: int = 360,
    ):
        self.range_min = range_min
        self.range_max = range_max
        self.n_beams = n_beams
        self.boxes: list[_Box] = []
        self.cylinders: list[_Cylinder] = []

    # ------------------------------------------------------------------
    # Scene authoring
    # ------------------------------------------------------------------

    def add_box(self, x: float, y: float, w: float, h: float) -> SyntheticLidar:
        self.boxes.append(_Box(x, y, w, h))
        return self

    def add_cylinder(self, x: float, y: float, r: float) -> SyntheticLidar:
        self.cylinders.append(_Cylinder(x, y, r))
        return self

    @classmethod
    def default_scene(cls) -> SyntheticLidar:
        """A 3 m cylinder dead ahead at 4 m, plus two side walls."""
        return (
            cls()
            .add_cylinder(4.0, 0.0, 0.5)
            .add_box(0.0, 3.0, 8.0, 0.2)
            .add_box(0.0, -3.0, 8.0, 0.2)
        )

    # ------------------------------------------------------------------
    # Ray casting
    # ------------------------------------------------------------------

    def _intersect_cylinder(self, dx: float, dy: float, c: _Cylinder) -> float | None:
        # |O + t*D - C|^2 = r^2,  O = (0,0),  D = (dx, dy), unit length.
        fx, fy = -c.x, -c.y
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - c.r * c.r)
        if disc < 0:
            return None
        sqrt_d = math.sqrt(disc)
        for t in (-b - sqrt_d, -b + sqrt_d):
            if t > 0:
                return t
        return None

    def _intersect_box(self, dx: float, dy: float, box: _Box) -> float | None:
        # Slab method on an axis-aligned box.
        x_min, x_max = box.x - box.w / 2, box.x + box.w / 2
        y_min, y_max = box.y - box.h / 2, box.y + box.h / 2

        def slab(d: float, lo: float, hi: float) -> tuple[float, float] | None:
            if abs(d) < 1e-9:
                if lo <= 0.0 <= hi:
                    return (-math.inf, math.inf)
                return None
            t1, t2 = lo / d, hi / d
            return (min(t1, t2), max(t1, t2))

        tx = slab(dx, x_min, x_max)
        ty = slab(dy, y_min, y_max)
        if tx is None or ty is None:
            return None
        t_enter = max(tx[0], ty[0])
        t_exit = min(tx[1], ty[1])
        if t_enter > t_exit or t_exit < 0:
            return None
        return max(t_enter, 0.0) or t_exit

    def _cast(self, angle_deg: float) -> float:
        th = math.radians(angle_deg)
        dx, dy = math.cos(th), math.sin(th)
        best = self.range_max
        for c in self.cylinders:
            t = self._intersect_cylinder(dx, dy, c)
            if t is not None and self.range_min <= t < best:
                best = t
        for b in self.boxes:
            t = self._intersect_box(dx, dy, b)
            if t is not None and self.range_min <= t < best:
                best = t
        return best

    # ------------------------------------------------------------------
    # ScanSource protocol
    # ------------------------------------------------------------------

    def read_scan(self) -> Scan:
        step = 360.0 / self.n_beams
        angles = [i * step for i in range(self.n_beams)]
        ranges = [self._cast(a) for a in angles]
        intens = [255] * self.n_beams
        return Scan(angles_deg=angles, ranges_m=ranges, intensities=intens)

    def close(self) -> None:  # pragma: no cover - no-op
        pass
