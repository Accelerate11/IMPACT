#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
world_filename="${XQ_COMPLEX_WORLD:-xq_complex_warehouse.sdf}"
lateral_offset_m="${XQ_COMPLEX_LATERAL_OFFSET:-0.68}"
enable_vertical_candidate="${XQ_COMPLEX_ENABLE_VERTICAL:-false}"
enable_diagonal_vertical_candidates="${XQ_COMPLEX_ENABLE_DIAGONAL_VERTICAL:-false}"
vertical_offset_m="${XQ_COMPLEX_VERTICAL_OFFSET:-0.70}"
integrity_information_memory_horizon_s="${XQ_COMPLEX_INFORMATION_MEMORY_HORIZON:-0.0}"
integrity_information_memory_max_frames="${XQ_COMPLEX_INFORMATION_MEMORY_MAX_FRAMES:-20.0}"
segment_goal_tolerance_m="${XQ_COMPLEX_SEGMENT_GOAL_TOLERANCE:-0.25}"
post_dynamic_static_confirmation_s="${XQ_COMPLEX_POST_DYNAMIC_STATIC_CONFIRMATION:-0.0}"
reversible_static_ttl_s="${XQ_COMPLEX_REVERSIBLE_STATIC_TTL:-0.0}"
maximum_rays="${XQ_COMPLEX_MAXIMUM_RAYS:-900}"
obstacle_enter_start_s="${XQ_COMPLEX_OBSTACLE_ENTER_START:-24.0}"
obstacle_enter_end_s="${XQ_COMPLEX_OBSTACLE_ENTER_END:-28.0}"
obstacle_leave_start_s="${XQ_COMPLEX_OBSTACLE_LEAVE_START:-44.0}"
obstacle_leave_end_s="${XQ_COMPLEX_OBSTACLE_LEAVE_END:-48.0}"
candidate_generation_mode="${XQ_CANDIDATE_GENERATION_MODE:-legacy}"
candidate_metric_source="${XQ_CANDIDATE_METRIC_SOURCE:-metadata}"
lattice_lateral_levels="${XQ_LATTICE_LATERAL_LEVELS:-5}"
lattice_vertical_levels="${XQ_LATTICE_VERTICAL_LEVELS:-2}"
task_progress_weight="${XQ_TASK_PROGRESS_WEIGHT:-0.85}"
task_map_age_time_constant_s="${XQ_TASK_MAP_AGE_TIME_CONSTANT_S:-20.0}"
research_energy_remaining="${XQ_RESEARCH_ENERGY_REMAINING:-32.0}"
utility_indifference_band="${XQ_UTILITY_INDIFFERENCE_BAND:-0.0}"
dynamic_path_query_mode="${XQ_DYNAMIC_PATH_QUERY_MODE:-forward_axis}"
minimum_dynamic_cluster_points="${XQ_MINIMUM_DYNAMIC_CLUSTER_POINTS:-1}"
dynamic_cluster_radius_m="${XQ_DYNAMIC_CLUSTER_RADIUS_M:-0.45}"
terminal_extension_mode="${XQ_TERMINAL_EXTENSION_MODE:-fixed}"
runtime_integrity_guard_mode="${XQ_RUNTIME_INTEGRITY_GUARD_MODE:-disabled}"
runtime_integrity_margin_m="${XQ_RUNTIME_INTEGRITY_MARGIN_M:-0.12}"
[[ "${world_filename}" == "$(basename -- "${world_filename}")" ]] || {
  echo "Complex visualization world must be an installed project-local filename." >&2; exit 2;
}
for required in DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR; do
  [[ -n "${!required:-}" ]] || { echo "WSLg variable ${required} is empty." >&2; exit 3; }
done
for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/${world_filename}" \
  "${workspace_root}/config/complex_live.rviz" \
  "${workspace_root}/config/complex_dynamic_thresholds.json" \
  "${workspace_root}/config/p13_gate_thresholds.json" \
  "${workspace_root}/evidence/P7/p7-calibration.json"; do
  [[ -f "${required}" ]] || { echo "Complex visualization dependency missing: ${required}" >&2; exit 2; }
done

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((170 + ($$ % 20)))}"
export ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
export GZ_PARTITION="xq_complex_live_$(date -u +%Y%m%dT%H%M%SZ)_$$"
export GZ_SIM_RESOURCE_PATH="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models:${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${workspace_root}/xq_install/xq_gz_bridge/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
unset LIBGL_ALWAYS_SOFTWARE MESA_LOADER_DRIVER_OVERRIDE GALLIUM_DRIVER EGL_PLATFORM QT_QPA_PLATFORM
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"

session_dir="/tmp/xq_complex_live_${ROS_DOMAIN_ID}_$$"
mkdir -p -- "${session_dir}"
declare -a pids=()
cleanup() {
  local status=$? pid round alive
  trap - EXIT INT TERM
  set +e
  for pid in "${pids[@]}"; do kill -INT -- "-${pid}" 2>/dev/null || true; done
  for round in 1 2 3 4 5; do
    alive=false
    for pid in "${pids[@]}"; do pgrep -g "${pid}" >/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do kill -TERM -- "-${pid}" 2>/dev/null || true; done
  exit "${status}"
}
trap cleanup EXIT INT TERM

world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/${world_filename}"
world_name="${world_filename%.sdf}"
setsid ros2 launch xq_sim_bringup xq_p13_flight.launch.py \
  world_file:="${world}" headless_rendering:=false \
  calibration_file:="${workspace_root}/evidence/P7/p7-calibration.json" \
  thresholds_file:="${workspace_root}/config/p13_gate_thresholds.json" \
  p12_thresholds_file:="${workspace_root}/config/complex_dynamic_thresholds.json" \
  p12_result_file:="${session_dir}/p12-result.json" \
  p13_result_file:="${session_dir}/p13-result.json" \
  planner_delay_ms:=50.0 voxel_size_m:=0.25 dynamic_ttl_s:=3.0 \
  integrity_information_memory_horizon_s:="${integrity_information_memory_horizon_s}" \
  integrity_information_memory_max_frames:="${integrity_information_memory_max_frames}" \
  dynamic_occupied_threshold:=0.35 dynamic_clear_threshold:=0.08 \
  static_confirmation_hits:=6 free_confirmation_rays:=3 \
  post_dynamic_static_confirmation_s:="${post_dynamic_static_confirmation_s}" \
  reversible_static_ttl_s:="${reversible_static_ttl_s}" \
  maximum_rays:="${maximum_rays}" \
  path_clearance_radius_m:=0.70 planning_lookahead_m:=4.0 clear_confirmation_s:=1.0 \
  mission_distance_m:=24.0 lateral_offset_m:="${lateral_offset_m}" lateral_candidate_shape:=challenge_then_center \
  enable_vertical_candidate:="${enable_vertical_candidate}" vertical_offset_m:="${vertical_offset_m}" \
  enable_diagonal_vertical_candidates:="${enable_diagonal_vertical_candidates}" \
  candidate_generation_mode:="${candidate_generation_mode}" \
  candidate_metric_source:="${candidate_metric_source}" \
  lattice_lateral_levels:="${lattice_lateral_levels}" \
  lattice_vertical_levels:="${lattice_vertical_levels}" \
  task_progress_weight:="${task_progress_weight}" \
  task_map_age_time_constant_s:="${task_map_age_time_constant_s}" \
  research_energy_remaining:="${research_energy_remaining}" \
  utility_indifference_band:="${utility_indifference_band}" \
  dynamic_path_query_mode:="${dynamic_path_query_mode}" \
  minimum_dynamic_cluster_points:="${minimum_dynamic_cluster_points}" \
  dynamic_cluster_radius_m:="${dynamic_cluster_radius_m}" \
  terminal_extension_mode:="${terminal_extension_mode}" \
  runtime_integrity_guard_mode:="${runtime_integrity_guard_mode}" \
  runtime_integrity_margin_m:="${runtime_integrity_margin_m}" \
  segment_goal_tolerance_m:="${segment_goal_tolerance_m}" \
  geometric_clearance_m:=0.82 fixed_buffer_m:=0.58 \
  protection_level_m:=0.10 required_margin_m:=0.06 maximum_speed_mps:=0.42 \
  maximum_acceleration_mps2:=0.8 rejected_candidate_retry_s:=1.0 \
  maximum_candidate_retries:=60 integrity_recovery_speed_mps:=0.08 \
  integrity_recovery_max_offset_m:=0.35 integrity_recovery_half_period_s:=3.0 \
  obstacle_x_m:=-4.5 obstacle_park_y_m:=6.6 obstacle_blocked_y_m:=0.0 obstacle_z_m:=1.0 \
  obstacle_enter_start_s:="${obstacle_enter_start_s}" obstacle_enter_end_s:="${obstacle_enter_end_s}" \
  obstacle_leave_start_s:="${obstacle_leave_start_s}" obstacle_leave_end_s:="${obstacle_leave_end_s}" \
  >"${session_dir}/launch.log" 2>&1 < /dev/null & pids+=("$!")
launch_pid="${pids[0]}"

deadline=$((SECONDS + 45))
until gz service -l 2>/dev/null | grep -Fq "/world/${world_name}/control"; do
  kill -0 "${launch_pid}" 2>/dev/null || { tail -n 120 "${session_dir}/launch.log" >&2; exit 4; }
  ((SECONDS < deadline)) || { echo "Complex Gazebo world did not become ready." >&2; exit 5; }
  sleep 0.25
done

setsid python3 "${workspace_root}/scripts/xq_p5_replay_visualizer.py" --ros-args -p use_sim_time:=true \
  >"${session_dir}/vehicle-viz.log" 2>&1 < /dev/null & pids+=("$!")
setsid ros2 run xq_autonomy xq_p11_replay_visualizer --ros-args \
  -p use_sim_time:=true -p show_complex_demo_overlay:=true \
  -p complex_scenario_name:="${world_name}" \
  >"${session_dir}/planning-viz.log" 2>&1 < /dev/null & pids+=("$!")
setsid ros2 run xq_autonomy xq_p13_replay_visualizer --ros-args -p use_sim_time:=true \
  >"${session_dir}/latency-viz.log" 2>&1 < /dev/null & pids+=("$!")

setsid env -u QT_QPA_PLATFORM -u EGL_PLATFORM nice -n 10 gz sim -g -v 3 \
  >"${session_dir}/gazebo-gui.log" 2>&1 < /dev/null & pids+=("$!")
gui_pid="${pids[${#pids[@]}-1]}"
sleep 4
kill -0 "${gui_pid}" 2>/dev/null || { tail -n 100 "${session_dir}/gazebo-gui.log" >&2; exit 6; }
camera_response="$(gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
  --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'pose: {position: {x: 0.0, y: 0.0, z: 31.0}, orientation: {x: 0.0, y: 0.70710678, z: 0.0, w: 0.70710678}} projection_type: "perspective"')"
grep -Fq 'data: true' <<<"${camera_response}" || echo "Warning: Gazebo camera auto-position was not acknowledged." >&2

setsid env -u QT_QPA_PLATFORM -u EGL_PLATFORM /opt/ros/humble/bin/rviz2 \
  -d "${workspace_root}/config/complex_live.rviz" --ros-args -p use_sim_time:=true \
  >"${session_dir}/rviz.log" 2>&1 < /dev/null & pids+=("$!")
rviz_pid="${pids[${#pids[@]}-1]}"
sleep 4
kill -0 "${rviz_pid}" 2>/dev/null || { tail -n 100 "${session_dir}/rviz.log" >&2; exit 7; }

echo "Complex live visualization is running."
renderer="$(glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p' | head -n 1)"
echo "GPU renderer: ${renderer:-unavailable}"
echo "Gazebo: ${world_name}; open roof, retained walls, racks, portals, clutter, two movers."
echo "RViz: LiDAR/static+dynamic voxels, FAST-LIO path, Frontier candidates, certified trajectory and latency safety."
echo "Mode: full normal flight stack; fault injection is disabled."
echo "Spatial candidates: vertical=${enable_vertical_candidate}, vertical_offset_m=${vertical_offset_m}."
echo "Observed-information memory: horizon=${integrity_information_memory_horizon_s}s, max_frames=${integrity_information_memory_max_frames}."
echo "Compositional candidates: diagonal_vertical=${enable_diagonal_vertical_candidates}."
echo "Research candidates: generation=${candidate_generation_mode}, metrics=${candidate_metric_source}, lattice=${lattice_lateral_levels}x${lattice_vertical_levels}."
echo "Minimum-intervention utility band: ${utility_indifference_band}."
echo "Dynamic safety path query: ${dynamic_path_query_mode}."
echo "Terminal extension policy: ${terminal_extension_mode}."
echo "Runtime integrity guard: ${runtime_integrity_guard_mode}, margin=${runtime_integrity_margin_m} m."
echo "Segment goal tolerance: ${segment_goal_tolerance_m} m."
echo "Post-dynamic static confirmation: ${post_dynamic_static_confirmation_s} s."
echo "Reversible static TTL: ${reversible_static_ttl_s} s."
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} GZ_PARTITION=${GZ_PARTITION}"
echo "Logs: ${session_dir}"
echo "Press Ctrl+C here to close only this visualization session."
while kill -0 "${gui_pid}" 2>/dev/null && kill -0 "${rviz_pid}" 2>/dev/null; do sleep 1; done
