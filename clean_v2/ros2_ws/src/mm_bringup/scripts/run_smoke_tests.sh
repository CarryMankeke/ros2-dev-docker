#!/usr/bin/env bash
# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2

set -euo pipefail

SMOKE_NS="${SMOKE_NS:-mm1}"
SMOKE_PREFIX="${SMOKE_PREFIX:-mm1_}"

tests=(
  "smoke_sim_time.py"
  "smoke_tf.py"
  "smoke_controllers.py"
  "smoke_cameras.py"
)

fail=0
for test in "${tests[@]}"; do
  echo "==> ${test}"
  if ros2 run mm_bringup "${test}" --ros-args -p namespace:="${SMOKE_NS}" -p prefix:="${SMOKE_PREFIX}"; then
    echo "PASS: ${test}"
  else
    echo "FAIL: ${test}"
    fail=1
  fi
done

if [[ "${fail}" -eq 0 ]]; then
  echo "PASS: smoke tests"
else
  echo "FAIL: smoke tests"
fi

exit "${fail}"
