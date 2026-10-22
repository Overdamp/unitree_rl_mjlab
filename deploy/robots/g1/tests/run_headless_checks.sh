#!/usr/bin/env bash
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
ort="$repo/deploy/thirdparty/onnxruntime-linux-x64-1.22.0"
mujoco="$repo/simulate/mujoco"
out="$(mktemp -d /tmp/g1-policy-checks.XXXXXX)"
trap 'rm -rf -- "$out"' EXIT
common=(
    -std=c++17 -O2
    "-I$repo/deploy/robots/g1/include" "-I$repo/deploy/include"
    "-I$ort/include" -I/opt/unitree_robotics/include
    -lyaml-cpp -lfmt "$ort/lib/libonnxruntime.so.1.22.0"
    "-Wl,-rpath,$ort/lib"
)
config="$repo/deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml"
policy="$repo/deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx"
scene="$repo/src/assets/robots/unitree_g1/xmls/scene_g1.xml"

g++ "$repo/deploy/robots/g1/tests/keyboard_observation_order.cpp" \
    "${common[@]}" -o "$out/observation_order"
env -u LD_LIBRARY_PATH "$out/observation_order" "$config" "$policy"

g++ "$repo/deploy/robots/g1/tests/headless_policy_smoke.cpp" \
    "${common[@]}" "-I$mujoco/include" "$mujoco/lib/libmujoco.so.3.3.6" \
    "-Wl,-rpath,$mujoco/lib" -o "$out/policy_smoke"
env -u LD_LIBRARY_PATH "$out/policy_smoke" "$scene" "$config" "$policy" fixed
for key in s a d q e; do
    env -u LD_LIBRARY_PATH "$out/policy_smoke" "$scene" "$config" "$policy" "$key"
done
if env -u LD_LIBRARY_PATH "$out/policy_smoke" "$scene" "$config" "$policy" broken; then
    echo "FAIL: the old observation bug unexpectedly survived the smoke scenario"
    exit 1
else
    result=$?
    if [[ "$result" != 2 ]]; then exit "$result"; fi
    echo "PASS: old configuration reproduces a fall"
fi
if env -u LD_LIBRARY_PATH "$out/policy_smoke" "$scene" "$config" "$policy" reset \
    "$repo/deploy/robots/g1/config/config.yaml"; then
    echo "Ordinary Reset also survived; inspect startup conditions if this changes."
else
    result=$?
    if [[ "$result" != 2 ]]; then exit "$result"; fi
    echo "Reproduced: ordinary Reset -> FixStand falls without stand_ready."
fi
if env -u LD_LIBRARY_PATH "$out/policy_smoke" "$scene" "$config" "$policy" fixed \
    "$repo/deploy/robots/g1/config/config.yaml"; then
    echo "FixStand held stand_ready; inspect startup conditions if this changes."
else
    result=$?
    if [[ "$result" != 2 ]]; then exit "$result"; fi
    echo "Reproduced: FixStand alone falls even from stand_ready; enter Velocity while paused."
fi
