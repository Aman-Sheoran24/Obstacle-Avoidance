# VFH+ Algorithm

This page walks through how
[`vfh/vfh_plus.py`](../src/obstacle_avoidance/vfh/vfh_plus.py) implements
the Vector Field Histogram Plus algorithm of:

> Iwan Ulrich and Johann Borenstein. **VFH+: Reliable Obstacle Avoidance for
> Fast Mobile Robots.** *Proc. IEEE ICRA*, 1998, pp. 1572-1577.

The PDF is mirrored at <https://cs.cmu.edu/~iwan/papers/vfh+.pdf>.

---

## Inputs and outputs

```
inputs    -> angles[N]  (degrees, sorted 0..360)
             ranges[N]  (metres; <=0 or >d_max treated as "no return")
             target_heading_deg
             current_heading_deg

output    -> theta_best_deg  OR  None  (brake / RTL on dead-end)
```

The planner is stateful — it remembers the previous binary histogram
(for hysteresis) and the previous chosen direction (for commitment).

## Stage A — polar obstacle density `H^p`

Divide 360 degrees into `n = 72` sectors of `alpha = 5` degrees. For every
LiDAR hit at angular bearing `beta` and distance `d <= d_max`:

```
m     = max(0, a - b * d^2)              # eq. 2
gamma = arcsin(r_safe / d)               # eq. 4
```

Add `m` to every sector whose centre lies within `+/- gamma` of `beta`
(the **enlargement angle** that accounts for the drone's radius plus a
safety margin `d_s`). `r_safe = drone_radius + d_s`.

Constants:

| Symbol  | Code field        | Default | Meaning                                      |
|---------|-------------------|---------|----------------------------------------------|
| `a`     | `VFHPlusConfig.a` | 197.0   | from `a - b * ((w_s - 1) / 2)^2 == 1`, w_s=29|
| `b`     | `VFHPlusConfig.b` | 1.0     |                                              |
| `d_max` | `d_max_m`         | 5.0 m   | planning horizon (sensor reach is 12 m)      |
| `r_safe`| `r_safe_m`        | 0.65 m  | drone radius + 0.3 m margin                  |

## Stage B — binary histogram `H^b` with hysteresis (eq. 7)

```
H^b[k] = 1                  if H^p[k] > tau_high
       = 0                  if H^p[k] < tau_low
       = H^b_prev[k]        otherwise
```

The two thresholds suppress flicker on the obstacle / free boundary.
The paper deliberately leaves `tau_low` and `tau_high` as
application-tuned. Defaults chosen here are `tau_low = a`,
`tau_high = 3 * a`; lower them if the drone reacts to obstacles too late.

## Stage C — masked histogram `H^m`

For a non-holonomic ground robot, sectors that fall inside the swept
circle of the vehicle's minimum-turning-radius `r_min` are masked off as
unreachable. **A multirotor that can strafe has `r_min = 0`**, so this
implementation treats `H^m == H^b`.

## Stage D — valley selection and cost function

1. Enumerate **valleys** — contiguous runs of `H^m[k] == 0`, treating the
   array as circular.
2. Classify each valley as wide (`>= s_max = 16` sectors -> 80 degrees)
   or narrow.
3. Build candidate steering directions:
   - **Narrow valley** -> centre: `c_n = (k_r + k_l) / 2`.
   - **Wide valley** -> two near-edge candidates plus the goal itself if
     it lies in the valley:

     ```
     c_r = k_r + s_max / 2
     c_l = k_l - s_max / 2
     c_t = k_t  if k_t in [c_r, c_l]
     ```
4. Score each candidate (eqs. 15-16):

   ```
   g(c) = mu1 * delta(c, k_t)
        + mu2 * delta(c, k_theta_i / alpha)
        + mu3 * delta(c, k_prev)
   ```

   where `delta(a, b)` is the wrap-aware angular distance in sector
   units.

5. Return the lowest-cost candidate.

### Cost weights and the necessary condition

Paper-recommended weights are **mu1 = 5, mu2 = 2, mu3 = 2** subject to
**mu1 > mu2 + mu3** (eq. 17) — without that inequality the planner can
indefinitely orbit obstacles instead of attempting the goal. The code
asserts this constraint in `VFHPlus.__init__`.

## Tuning notes

- **Drone oscillates between two valleys.** Increase `mu3` (commitment to
  the previous direction) by 1 at a time, keeping `mu1 > mu2 + mu3`.
- **Drone hugs obstacles too closely.** Raise `r_safe_m` by 0.1 m.
- **Drone reacts too late.** Lower `tau_low` and `tau_high` proportionally.
- **VFH+ output is None even with visible gaps.** The hysteresis state may
  be "stuck" — clear `vfh.h_b_prev` between scenes, or raise `tau_low`.
- **CPU pegged on the Orin Nano.** `_polar_density` is already vectorised;
  the remaining cost is the candidate loop, which iterates over fewer
  than 20 directions per step.
