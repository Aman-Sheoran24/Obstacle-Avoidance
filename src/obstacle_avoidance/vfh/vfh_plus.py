"""VFH+ (Ulrich & Borenstein, ICRA 1998) for a 360-degree 2D LiDAR.

Pipeline (paper section / equation references inline):

* Stage A — polar obstacle density H^p (eq. 2-4)
* Stage B — binary histogram H^b with hysteresis (eq. 7)
* Stage C — masked histogram H^m (multirotor strafes, so masked == binary)
* Stage D — valley enumeration + cost-function selection (eq. 12-17)

The cost function ``g(c) = mu1 * d(c, k_t) + mu2 * d(c, k_theta) + mu3 * d(c,
k_prev)`` requires ``mu1 > mu2 + mu3`` (eq. 17) so goal attraction dominates.
"""

from __future__ import annotations

import numpy as np

from ..config import VFHPlusConfig


class VFHPlus:
    """Stateful VFH+ planner — keeps the previous binary histogram + heading."""

    def __init__(
        self,
        n_sectors: int = 72,
        alpha_deg: float = 5.0,
        r_safe: float = 0.65,
        d_max: float = 5.0,
        a: float = 197.0,
        b: float = 1.0,
        tau_low: float = 197.0,
        tau_high: float = 591.0,
        s_max: int = 16,
        mu1: float = 5.0,
        mu2: float = 2.0,
        mu3: float = 2.0,
    ):
        if not np.isclose(n_sectors * alpha_deg, 360.0):
            raise ValueError("n_sectors * alpha_deg must equal 360")
        if not (mu1 > mu2 + mu3):
            raise ValueError("VFH+ requires mu1 > mu2 + mu3 (eq. 17)")
        self.n = n_sectors
        self.alpha = alpha_deg
        self.r_safe = r_safe
        self.d_max = d_max
        self.a = a
        self.b = b
        self.tau_low = tau_low
        self.tau_high = tau_high
        self.s_max = s_max
        self.mu = (mu1, mu2, mu3)

        # State.
        self.h_b_prev = np.zeros(n_sectors, dtype=np.int8)
        self.k_prev = 0
        self._last_hp: np.ndarray | None = None

    @classmethod
    def from_config(cls, cfg: VFHPlusConfig) -> VFHPlus:
        return cls(
            n_sectors=cfg.n_sectors,
            alpha_deg=cfg.alpha_deg,
            r_safe=cfg.r_safe_m,
            d_max=cfg.d_max_m,
            a=cfg.a,
            b=cfg.b,
            tau_low=cfg.tau_low,
            tau_high=cfg.tau_high,
            s_max=cfg.s_max,
            mu1=cfg.mu1,
            mu2=cfg.mu2,
            mu3=cfg.mu3,
        )

    # ------------------------------------------------------------------
    # Public step
    # ------------------------------------------------------------------

    def step(
        self,
        angles_deg,
        ranges_m,
        target_heading_deg: float,
        current_heading_deg: float,
    ) -> float | None:
        """Return the best steering direction in degrees, or None on dead-end."""
        Hp = self._polar_density(
            np.asarray(angles_deg, dtype=float), np.asarray(ranges_m, dtype=float)
        )
        self._last_hp = Hp

        # Stage B — hysteresis.
        Hb = np.where(Hp > self.tau_high, 1, np.where(Hp < self.tau_low, 0, self.h_b_prev)).astype(
            np.int8
        )
        self.h_b_prev = Hb
        Hm = Hb  # Stage C is a no-op for an omnidirectional multirotor.

        # Stage D.
        k_t = int(round(target_heading_deg / self.alpha)) % self.n
        k_theta = int(round(current_heading_deg / self.alpha)) % self.n
        candidates = self._candidates(Hm, k_t)
        if not candidates:
            return None

        def d_sector(a: int, b: int) -> int:
            d = abs(a - b)
            return min(d, self.n - d)

        mu1, mu2, mu3 = self.mu
        # ``min(... , c)`` ties on cost are broken by the candidate index,
        # which is deterministic and goal-favouring (lower sector wins).
        _, k_best = min(
            (
                mu1 * d_sector(c, k_t)
                + mu2 * d_sector(c, k_theta)
                + mu3 * d_sector(c, self.k_prev),
                c,
            )
            for c in candidates
        )
        self.k_prev = k_best
        return (k_best * self.alpha) % 360.0

    # ------------------------------------------------------------------
    # Stage A — vectorised polar density
    # ------------------------------------------------------------------

    def _polar_density(self, angles: np.ndarray, ranges: np.ndarray) -> np.ndarray:
        Hp = np.zeros(self.n, dtype=float)
        if angles.size == 0:
            return Hp

        valid = (ranges > 0.0) & (ranges <= self.d_max)
        if not np.any(valid):
            return Hp
        beta = angles[valid]
        d = ranges[valid]

        # eq. 2: m = max(0, a - b * d^2)
        m = np.maximum(0.0, self.a - self.b * d * d)
        # eq. 4: gamma = arcsin(r_safe / d), clamped to [0, 90 deg].
        gamma = np.degrees(np.arcsin(np.clip(self.r_safe / np.maximum(d, 1e-6), 0.0, 1.0)))

        sector_centres = np.arange(self.n) * self.alpha
        # Wrap-aware angular distance from every (point, sector) pair.
        diff = (sector_centres[None, :] - beta[:, None] + 180.0) % 360.0 - 180.0
        within = np.abs(diff) <= gamma[:, None]
        # Sum point magnitude into every sector it enlarges into.
        Hp += (m[:, None] * within).sum(axis=0)
        return Hp

    # ------------------------------------------------------------------
    # Stage D helper — wrap-aware valley enumeration
    # ------------------------------------------------------------------

    def _candidates(self, Hm: np.ndarray, k_t: int) -> list[int]:
        if Hm.sum() == 0:
            return [k_t]  # entirely free — head straight at the goal
        if Hm.sum() == self.n:
            return []  # entirely blocked

        ext = np.concatenate([Hm, Hm])  # wrap-aware
        valleys: list[tuple[int, int]] = []
        start: int | None = None
        for i, v in enumerate(ext):
            if v == 0 and start is None:
                start = i
            elif v == 1 and start is not None:
                valleys.append((start, i - 1))
                start = None
        if start is not None:
            valleys.append((start, len(ext) - 1))

        out: list[int] = []
        seen: set[tuple[int, int]] = set()
        for kr, kl in valleys:
            key = (kr % self.n, kl % self.n)
            if key in seen:
                continue
            seen.add(key)
            width = kl - kr + 1
            if width >= self.s_max:
                out.append((kr + self.s_max // 2) % self.n)
                out.append((kl - self.s_max // 2) % self.n)
                if kr <= k_t <= kl:
                    out.append(k_t)
            else:
                out.append(((kr + kl) // 2) % self.n)
        return out

    # ------------------------------------------------------------------
    # Debug accessors used by the pipeline for adaptive speed control.
    # ------------------------------------------------------------------

    def last_density_peak(self) -> float | None:
        return float(self._last_hp.max()) if self._last_hp is not None else None

    def last_density_at(self, theta_deg: float) -> float:
        if self._last_hp is None:
            return 0.0
        k = int(round(theta_deg / self.alpha)) % self.n
        return float(self._last_hp[k])
