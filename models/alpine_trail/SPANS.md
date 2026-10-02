# alpine_trail cable spans (Phase 1)

Every valid ice-block position is `(span_id, cable_id, t)` with `t ∈ [0, 1]`
along the span. Geometry is implemented in `scripts/ice_cable_geometry.py`.
Numbers below match `py -3 scripts/ice_cable_geometry.py`.

Visuals, collisions, worlds, and Nav2 are unchanged. This file is the human
table only.

## Transform

The model is **pre-inclined** (ground `z ≈ tan(0.07)·x` in the **model** frame).
The world include has **no extra RPY**:

```
WORLD_OFFSET = (20.0, -30.0, 0.0)
world = model + WORLD_OFFSET
```

| | model | world |
|---|---|---|
| origin | `(0, 0, 0)` | `(20, -30, 0)` |
| RPY | `(0, 0, 0)` | `(0, 0, 0)` |

`RAMP_PITCH = 0.07 rad`. `CABLE_RADIUS = 0.17 m` (visual diameter 0.34 m).

## Towers (model `y = 0`)

| tower | model x | mesh centreline z | world xyz |
|---|---|---|---|
| T0 | 8.0 | 6.938 | `(28.000, -30.000, 6.938)` |
| T1 | 24.0 | 8.153 | `(44.000, -30.000, 8.153)` |
| T2 | 40.0 | 9.305 | `(60.000, -30.000, 9.305)` |
| T3 | 54.0 | 10.266 | `(74.000, -30.000, 10.266)` |

`z` is the `cables.obj` **outer-vertex envelope midpoint** at that x
(range ≈ 0.34 m, so centre ≈ mid of min/max). That fit is used instead of
`z = tan(0.07)·x + 6.50`, which sits ~0.12 m above T0.

## Cables (locked names)

Mesh AABB `y ∈ [-0.92, 0.92]`. Subtracting the 0.17 m radius gives centreline
`y = ±0.75` (measured ring centroids are exactly ±0.75). Names kept.

| cable_id | model y | world y | notes |
|---|---|---|---|
| `cable_left` | −0.75 | **−30.75** | further from the Nav2 corridor |
| `cable_right` | +0.75 | **−29.25** | closer to the corridor; still `\|y\| ≫ 12` |

## Centreline

`t` is **linear in model x** from the downhill tower (`t = 0`) to the uphill
tower (`t = 1`). Local +X is the uphill tangent. Roll = 0, yaw = 0.

```
x(t) = x0 + t (x1 − x0)
z(t) = (1−t) z0 + t z1 − 4 s t (1−t)
pitch(t) = atan(dz/dx)
```

`s` is vertical sag (metres below the tower chord at `t = 0.5`), fitted to
`cables.obj` ring-centroid z at each span midpoint. All `s ≤ 0.35 m`.

| span | towers | model x | z0 → z1 | sag s | chord |
|---|---|---|---|---|---|
| `span_01` | T0 → T1 | 8 → 24 | 6.938 → 8.153 | 0.267 m | 16.046 m |
| `span_02` | T1 → T2 | 24 → 40 | 8.153 → 9.305 | 0.328 m | 16.041 m |
| `span_03` | T2 → T3 | 40 → 54 | 9.305 → 10.266 | 0.339 m | 14.033 m |

Pitch at `t = 0.5` equals the chord slope (~0.07 rad plus the small difference
between mesh tower z and a pure 0.07 ramp).

## Span endpoints

Both cables share x, z, and pitch; only y differs.

### `span_01` (T0 → T1)

| cable | t | model xyz | world xyz | pitch (rad) |
|---|---|---|---|---|
| `cable_left` | 0 | `(8.000, −0.750, 6.938)` | `(28.000, −30.750, 6.938)` | 0.00919 |
| `cable_left` | 1 | `(24.000, −0.750, 8.153)` | `(44.000, −30.750, 8.153)` | 0.14173 |
| `cable_right` | 0 | `(8.000, +0.750, 6.938)` | `(28.000, −29.250, 6.938)` | 0.00919 |
| `cable_right` | 1 | `(24.000, +0.750, 8.153)` | `(44.000, −29.250, 8.153)` | 0.14173 |

### `span_02` (T1 → T2)

| cable | t | model xyz | world xyz | pitch (rad) |
|---|---|---|---|---|
| `cable_left` | 0 | `(24.000, −0.750, 8.153)` | `(44.000, −30.750, 8.153)` | −0.01000 |
| `cable_left` | 1 | `(40.000, −0.750, 9.305)` | `(60.000, −30.750, 9.305)` | 0.15280 |
| `cable_right` | 0 | `(24.000, +0.750, 8.153)` | `(44.000, −29.250, 8.153)` | −0.01000 |
| `cable_right` | 1 | `(40.000, +0.750, 9.305)` | `(60.000, −29.250, 9.305)` | 0.15280 |

### `span_03` (T2 → T3)

| cable | t | model xyz | world xyz | pitch (rad) |
|---|---|---|---|---|
| `cable_left` | 0 | `(40.000, −0.750, 9.305)` | `(60.000, −30.750, 9.305)` | −0.02821 |
| `cable_left` | 1 | `(54.000, −0.750, 10.266)` | `(74.000, −30.750, 10.266)` | 0.16401 |
| `cable_right` | 0 | `(40.000, +0.750, 9.305)` | `(60.000, −29.250, 9.305)` | −0.02821 |
| `cable_right` | 1 | `(54.000, +0.750, 10.266)` | `(74.000, −29.250, 10.266)` | 0.16401 |

Endpoint `t = 0` / `t = 1` are **not valid** ice sites (tower heads).

## Keep-outs and valid `t`

Default (stay off tower heads): **`t ∈ [0.15, 0.85]`**.

Extra cabin keep-out: **1.5 m along the cable chord**, converted with
`Δt = 1.5 / chord_length`. Invalid when `|t − t_cabin| ≤ Δt`.

| cabin | model pose (collision) | span | `t_cabin` | `Δt` | excluded t |
|---|---|---|---|---|---|
| `cabin_0` | `(18.000, 0, 5.559)` | `span_01` | 0.625000 | 0.093481 | `[0.531519, 0.718481]` |
| `cabin_1` | `(46.000, 0, 7.507)` | `span_03` | 0.428571 | 0.106891 | `[0.321680, 0.535463]` |

`is_valid(span_id, cable_id, t)` is False inside those keep-outs
(`|t − t_cabin| < Δt`), outside `[0.15, 0.85]`, or for unknown ids.
The 1.5 m boundary itself is allowed.

| span | valid t |
|---|---|
| `span_01` | `[0.150000, 0.531519] ∪ [0.718481, 0.850000]` |
| `span_02` | `[0.150000, 0.850000]` |
| `span_03` | `[0.150000, 0.321680] ∪ [0.535463, 0.850000]` |

## Phase-3 target

Manual ice sleeve at the centre of `span_01`, **`cable_right`**, **`t = 0.5`**
(also listed for `cable_left`). This `t` is **valid** (2.0 m of model-x from
`cabin_0`, outside the 1.5 m keep-out).

| | `cable_right` (target) | `cable_left` |
|---|---|---|
| id | `span_01` / `cable_right` / `t=0.5` | `span_01` / `cable_left` / `t=0.5` |
| model xyz | `(16.0000, 0.7500, 7.2785)` | `(16.0000, −0.7500, 7.2785)` |
| world xyz | **`(36.0000, −29.2500, 7.2785)`** | `(36.0000, −30.7500, 7.2785)` |
| world RPY (rad) | **`(0.000000, 0.075792, 0.000000)`** | `(0.000000, 0.075792, 0.000000)` |
| `is_valid` | `True` | `True` |

SDF / `set_pose` for the Phase-3 block (world, metres, radians):

```
36.0000  -29.2500  7.2785  0  0.075792  0
```

Local +X of the ice sleeve is the cable tangent (uphill / increasing x).
Pitch 0.075792 rad is the `span_01` chord slope; sag derivative is zero at
`t = 0.5`.
