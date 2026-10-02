# Husky world verification task

Purpose: confirm `snow_forest` is set up correctly — free Nav2 on the corridor,
ramp climb works, and sensors can see objects (trees / thick alpine cables).

## 0) Start sim

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export ROS_LOCALHOST_ONLY=1
source ~/41068_ws/install/setup.bash

ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \
  world:=snow_forest slam:=true nav2:=true rviz:=true
```

Wait until Nav2 is **active** and `/husky1/scan_filtered` is publishing:
```bash
ros2 topic hz /husky1/scan_filtered
```

## 1) Automated mission (recommended)

In a second WSL terminal:

```bash
export ROS_LOCALHOST_ONLY=1
source ~/41068_ws/install/setup.bash
ros2 launch 41068_ignition_bringup verify_world_mission.launch.py
```

What it does:

| Step | Goal | Checks |
|------|------|--------|
| flat_open | (8, 0) | free flat snow |
| pre_ramp | (18, 0) | approach ramp |
| ramp_entry | (25, 0) | cross edge (no fake wall) |
| ramp_mid | (40, 0) | climb |
| view_alpine | (45, 0) facing −Y | **lidar hits** toward alpine/trees |
| ramp_upper | (60, 0) | continue climb |
| near_rr4 | (72, 0) | near trimmed summit |

Watch the mission terminal for `PASS` / `FAIL` and a final **VERIFY SUMMARY**.

## 2) Manual checks in RViz / Gazebo (tick these)

### Free space / no false obstacles
- [ ] Grey map includes spawn; no “out of bounds” spam
- [ ] Corridor `y≈0` looks free (trees only at |y|≳20)
- [ ] Ramp is **not** a solid wall in costmap (scan_filtered working)
- [ ] Small Nav2 goals on `y=0` succeed: (10,0) → (25,0) → (40,0)

### Object detection
- [ ] Lidar shows trees as obstacles in costmap
- [ ] At mid-ramp, looking right (−Y): LaserScan hits toward alpine (~15–40 m)
- [ ] Camera panel: thick icy cables / trees visible (RGB sanity check)

### Physics / world
- [ ] Husky stays on snow (does **not** fly — no vz altitude-follow)
- [ ] Alpine at pose `20 -30 0`; drive lane |y|&lt;12 clear
- [ ] World ends near RR4 (~x=75–80), no old long alpine past x=90

## 3) If something fails

| Symptom | Likely cause |
|---------|----------------|
| Out of bounds | Wait for SLAM map / landmarks |
| Stuck at x≈20 wall | `scan_filtered` missing or filter not running |
| Goal under snow | Use TopDownOrtho; click grey map on y=0 |
| DETECT FAIL | Face −Y; check alpine included; use raw `scan` if filtered too aggressive |
| Husky flies | Altitude-follow must NOT run on Husky |

## 4) Pass criteria

World is “properly configured” when:
1. All NAV waypoints PASS, and
2. DETECT alpine/trees PASS, and
3. Manual ticks above are OK.

## Optional: Parrot short cable tour

Do **not** enable slam/nav2 (avoids altitude-follow fighting the tour).

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py \
  world:=snow_forest rviz:=true
# rabbits default off on parrot launch (keeps Gazebo responsive)

# second terminal
ros2 launch 41068_ignition_bringup parrot_cable_tour.launch.py
```

Expect climb pose **z** to rise (needs OdometryPublisher `dimensions=3`). Then `TOUR SUMMARY` with `"pass": true` after climb → deploy_roof → span01_view → return_roof.

## Optional: Parrot Phase 2 (Nav2 + altitude-follow)

Same XY loop as Phase 1; height is terrain hover only (not roof/cable).
Do **not** run Phase 1 tour at the same time.

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py \
  world:=snow_forest rviz:=true slam:=true nav2:=true

# wait until map looks usable, then:
ros2 launch 41068_ignition_bringup parrot_nav2_cable_tour.launch.py
```

Expect `TOUR SUMMARY` with `"phase": 2` and `"pass": true`.

## Optional: Husky Phase 2 (Nav2 short cable loop)

Same *story* as Parrot Phase 2 cable view, ground-safe corridor only
(`span01_approach` → `span01_view` at y=0). No deploy-station legs.
Do **not** run `husky_ice_inspect` at the same time.

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \
  world:=snow_forest rviz:=true slam:=true nav2:=true

# wait until map looks usable, then:
ros2 launch 41068_ignition_bringup husky_nav2_cable_tour.launch.py
```

Expect `TOUR SUMMARY` with `"phase": 2`, `"robot": "husky"`, `"pass": true`.
After `span01_view`, log should show `pose_source=tf_map` and `corridor_ok=True`
(map TF, not Gazebo odometry — see ISSUE 16). Restart Husky launch after sync so
updated `nav2_params_husky1.yaml` loads (ISSUE 17).

## Quick test: Parrot find ice → Husky Nav2 (dual robot)

Terminal 1 — Husky `(0,0)` + Parrot on deploy roof; Husky Nav2 only on Parrot:

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_dual_ice.launch.py \
  world:=snow_forest rviz:=true
```

Wait until Husky map covers spawn (no costmap OOB spam).

Terminal 2 — seed ice on cables, patrol, hover, dispatch Husky:

```bash
ros2 launch 41068_ignition_bringup ice_aerial_husky_handoff.launch.py \
  seed:=42 block_count:=1 span_ids:=span_01
```

Ice is forced onto **span_01** (matches Parrot `span01_view`). Expect
`HANDOFF SUMMARY` with `"pass": true`. Detection uses ice ground truth
(quick test); swap for camera/thermal later.

