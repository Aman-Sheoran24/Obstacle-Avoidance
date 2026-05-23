"""Matplotlib helpers for debugging the VFH+ planner."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .vfh_plus import VFHPlus


def plot_polar(vfh: VFHPlus, theta_best_deg: float | None, show: bool = True):
    """Polar plot: H^p magnitude, H^b binary overlay, and chosen heading."""
    import matplotlib.pyplot as plt

    Hp = vfh._last_hp
    if Hp is None:
        raise RuntimeError("call VFHPlus.step() first")

    sector_angles = np.deg2rad(np.arange(vfh.n) * vfh.alpha)
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)

    ax.bar(sector_angles, Hp, width=np.deg2rad(vfh.alpha), alpha=0.6, label="H^p")
    Hb = vfh.h_b_prev
    blocked = Hb == 1
    if np.any(blocked):
        ax.bar(
            sector_angles[blocked],
            np.full(blocked.sum(), Hp.max() * 1.05),
            width=np.deg2rad(vfh.alpha),
            color="red",
            alpha=0.25,
            label="H^b == 1",
        )
    if theta_best_deg is not None:
        ax.plot(
            [np.deg2rad(theta_best_deg)] * 2,
            [0, Hp.max() * 1.1],
            "g-",
            linewidth=3,
            label=f"chosen heading: {theta_best_deg:.0f} deg",
        )
    ax.set_title("VFH+ polar histogram")
    ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.0))
    if show:
        plt.show()
    return fig
