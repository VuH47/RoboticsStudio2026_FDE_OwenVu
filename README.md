# MinhandOwen Robotics Studio Workspace

Curated workspace containing only active and verified assets for **Minh (Vu)** and **Owen**.


## Team Division
**Owen - UI & Decision:**
  - UI:
  - Decision: `scripts/mission_gate.py`, `scripts/ice_aerial_husky_handoff.py`, `scripts/demo_hazard_stub.py`- 
- This will need to adjust when Owen join new repo. 

**Minh (Vu) - Perception & Simulation Architecture:**
  - Perception: `scripts/ice_rgbd_detector.py`, `scripts/test_ice_rgbd_detector.py`, `launch/ice_detector.launch.py`
  - Sim World: `worlds/snow_forest.sdf`, `scripts/ice_cable_geometry.py`, `scripts/ice_scenario_manager.py`

- **Course Milestones:**
  - Rubric & verification checklist: `verify_world_task.md`
  - Autonomous verification mission: `launch/verify_world_mission.launch.py`
  - Quantitative benchmark: `launch/ice_evaluator.launch.py`

## Quick Launch
```bash
# 1. Master simulation in snow_forest
ros2 launch 41068_ignition_bringup 41068_ignition_dual_ice.launch.py world:=snow_forest rviz:=true rabbits:=false

# 2. Deploy ice on cable
ros2 launch 41068_ignition_bringup ice_scenario.launch.py world:=snow_forest seed:=42 block_count:=1 span_ids:=span_01

# 3. Perception node
ros2 launch 41068_ignition_bringup ice_detector.launch.py

# 4. Operator UI (Owen)
ros2 launch 41068_ignition_bringup 41068_operator_ui.launch.py
```
