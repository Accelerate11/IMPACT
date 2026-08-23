#!/usr/bin/env bash
set -euo pipefail

output_file="${1:-/dev/null}"
world_name="${2:-xq_p5_structured_room}"
ceiling_node="${3:-xq_office_shell::xq_ceiling_link}"

scene_response="$(
  gz service -s "/world/${world_name}/scene/info" \
    --reqtype gz.msgs.Empty --reptype gz.msgs.Scene \
    --timeout 5000 --req ''
)"
grep -Fq 'name: "xq_ceiling_link"' <<<"${scene_response}" || {
  echo "Gazebo GUI scene is missing xq_ceiling_link." >&2
  exit 1
}

transparent_response="$(
  gz service -s /gui/view/transparent \
    --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
    --timeout 5000 --req "data: \"${ceiling_node}\""
)"
printf 'ceiling_transparent_response=%s\n' "${transparent_response}" >"${output_file}"
grep -Fq 'data: true' <<<"${transparent_response}" || {
  echo "Gazebo GUI could not make xq_ceiling_link transparent." >&2
  exit 1
}

camera_response="$(
  gz service -s /gui/move_to/pose \
    --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req 'pose: {position: {x: 0.0, y: -0.5, z: 17.0}, orientation: {x: 0.0, y: 0.70710678, z: 0.0, w: 0.70710678}} projection_type: "perspective"'
)"
printf 'camera_pose_response=%s\n' "${camera_response}" >>"${output_file}"
grep -Fq 'data: true' <<<"${camera_response}" || {
  echo "Gazebo GUI could not apply the overhead camera pose." >&2
  exit 1
}

echo "PASS: ceiling transparent in the GUI; overhead camera pose applied." >>"${output_file}"
