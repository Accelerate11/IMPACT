#!/usr/bin/env bash
set -euo pipefail

node_executable=${1:?usage: test_shutdown_race.sh /path/to/xq_gz_bridge_node}
test_root=$(mktemp -d /tmp/xq_gz_bridge_shutdown.XXXXXX)
node_pid=""
publisher_pid=""

cleanup_processes() {
  set +e
  if [[ -n "${publisher_pid}" ]] && kill -0 "${publisher_pid}" 2>/dev/null; then
    kill -TERM -- "-${publisher_pid}" 2>/dev/null || kill -TERM "${publisher_pid}" 2>/dev/null || true
  fi
  if [[ -n "${node_pid}" ]] && kill -0 "${node_pid}" 2>/dev/null; then
    kill -KILL "${node_pid}" 2>/dev/null || true
  fi
  wait 2>/dev/null || true
}

cleanup_all() {
  cleanup_processes
  rm -rf "${test_root}"
}
trap cleanup_all EXIT INT TERM

for iteration in $(seq 1 10); do
  export ROS_DOMAIN_ID=$((70 + iteration))
  export GZ_PARTITION="xq_shutdown_${$}_${iteration}"
  log_file="${test_root}/node_${iteration}.log"

  "${node_executable}" --ros-args -p publish_ground_truth:=false >"${log_file}" 2>&1 &
  node_pid=$!

  ready=0
  for _ in $(seq 1 100); do
    if grep -q "XQ bridge ready" "${log_file}"; then
      ready=1
      break
    fi
    if ! kill -0 "${node_pid}" 2>/dev/null; then
      cat "${log_file}"
      echo "bridge exited before becoming ready (iteration ${iteration})" >&2
      exit 1
    fi
    sleep 0.02
  done
  if [[ "${ready}" -ne 1 ]]; then
    cat "${log_file}"
    echo "bridge readiness timeout (iteration ${iteration})" >&2
    exit 1
  fi

  # Establish the transport connection and complete at least one callback
  # before stressing the exact shutdown boundary.
  gz topic -t /clock -m gz.msgs.Clock -p "sim { sec: 1 nsec: 1 }" >/dev/null

  # Publish in a dedicated process group so cleanup never targets unrelated
  # Gazebo processes. Repeated CLI publications keep the callback active near
  # the rclcpp shutdown boundary.
  setsid bash -c '
    target_pid=$1
    while kill -0 "${target_pid}" 2>/dev/null; do
      gz topic -t /clock -m gz.msgs.Clock -p "sim { sec: 1 nsec: 2 }" >/dev/null 2>&1 || true
    done
  ' _ "${node_pid}" &
  publisher_pid=$!

  sleep 0.08
  kill -INT "${node_pid}"

  stopped=0
  for _ in $(seq 1 150); do
    if ! kill -0 "${node_pid}" 2>/dev/null; then
      stopped=1
      break
    fi
    sleep 0.02
  done
  if [[ "${stopped}" -ne 1 ]]; then
    cat "${log_file}"
    echo "bridge did not stop after SIGINT (iteration ${iteration})" >&2
    exit 1
  fi

  set +e
  wait "${node_pid}"
  node_status=$?
  set -e
  node_pid=""

  if kill -0 "${publisher_pid}" 2>/dev/null; then
    kill -TERM -- "-${publisher_pid}" 2>/dev/null || true
  fi
  wait "${publisher_pid}" 2>/dev/null || true
  publisher_pid=""

  if [[ "${node_status}" -ne 0 ]]; then
    cat "${log_file}"
    echo "bridge returned ${node_status} after SIGINT (iteration ${iteration})" >&2
    exit 1
  fi
  if grep -Eqi "rmw handle is invalid|terminate called|Aborted|core dumped" "${log_file}"; then
    cat "${log_file}"
    echo "shutdown race signature found (iteration ${iteration})" >&2
    exit 1
  fi
done

echo "shutdown race regression: 10/10 clean exits"
