#!/usr/bin/env bash
# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2

set -euo pipefail

SMOKE_NS="${SMOKE_NS:-mm1}"
SMOKE_PREFIX="${SMOKE_PREFIX:-mm1_}"
SMOKE_STARTUP_DELAY="${SMOKE_STARTUP_DELAY:-8}"

tests=(
  "smoke_sim_time.py"
  "smoke_tf.py"
  "smoke_controllers.py"
  "smoke_cameras.py"
)

fail=0

echo "Esperando ${SMOKE_STARTUP_DELAY}s para estabilizar la simulacion..."
sleep "${SMOKE_STARTUP_DELAY}"
for test in "${tests[@]}"; do
  echo "==> ${test}"
  case "${test}" in
    smoke_sim_time.py)
      args=(--ros-args -p namespace:="${SMOKE_NS}" -p prefix:="${SMOKE_PREFIX}" -p timeout_sec:=10.0 -p min_samples:=3)
      ;;
    smoke_tf.py)
      args=(--ros-args -p prefix:="${SMOKE_PREFIX}" -p timeout_sec:=5.0 -p sample_sec:=4.0)
      ;;
    smoke_controllers.py)
      args=(--ros-args -p namespace:="${SMOKE_NS}" -p timeout_sec:=10.0)
      ;;
    smoke_cameras.py)
      args=(--ros-args -p namespace:="${SMOKE_NS}" -p prefix:="${SMOKE_PREFIX}" -p timeout_sec:=10.0)
      ;;
    *)
      args=(--ros-args -p namespace:="${SMOKE_NS}" -p prefix:="${SMOKE_PREFIX}")
      ;;
  esac

  if ros2 run mm_bringup "${test}" "${args[@]}"; then
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
