cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros_ign_interfaces REQUIRED)

install(
  DIRECTORY config launch models urdf_husky worlds urdf_parrot maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/dynamic_world_demo.py scripts/basic_autonomy_demo.py
           scripts/deploy_patrol_mission.py
           scripts/terrain_perception.py scripts/scan_ground_filter.py
           scripts/terrain_altitude_follow.py scripts/verify_world_mission.py
           scripts/mission_gate.py scripts/operator_console.py
           scripts/path_preview_viz.py scripts/camera_hud_overlay.py
           scripts/demo_hazard_stub.py
           scripts/ice_cable_geometry.py
           scripts/ice_scenario_manager.py
           scripts/ice_evaluator.py
           scripts/husky_ice_inspect.py
           scripts/husky_nav2_cable_tour.py
           scripts/snow_rabbit_hopper.py
           scripts/parrot_cable_tour.py
           scripts/parrot_full_cable_tour.py
           scripts/parrot_nav2_cable_tour.py
           scripts/parrot_teleop.py
           scripts/ice_aerial_husky_handoff.py
           scripts/ice_unified_handoff.py
           scripts/ice_random_confirm.py
           scripts/ice_perception_confirm.py
           scripts/unified_tf_bridge.py
           scripts/monotonic_clock.py
           scripts/ice_rgbd_detector.py
           scripts/save_slam_map.py
           scripts/husky_ice_vision_core.py
           scripts/husky_ice_visual_confirm.py
           scripts/test_husky_ice_visual_confirm.py
           scripts/detection_viewer.py
           scripts/detection_viewer_core.py
           scripts/test_detection_viewer.py
           scripts/test_husky_pose_correction.py
           scripts/test_scan_ground_filter_pitch.py
           scripts/ice_target_tracker.py
           scripts/test_ice_target_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
