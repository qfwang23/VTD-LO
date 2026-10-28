#!/usr/bin/env bash
set -euo pipefail

#############################
# Config (modify if needed)
#############################
LAUNCH_PKG="vtd_lo"          # ROS package
LAUNCH_FILE="kitti.launch"   # Launch file

DATASET_DIR="$HOME/DATASET/KITTI"   # KITTI bags 00.bag through 10.bag
OUT_DIR="$HOME/VTD-LO/result/vtd_lo_logs_kitti"   # Per-sequence log directory

# TUM trajectory output directory
TUM_ROOT="$HOME/VTD-LO/result/tum"

# Module switches
USE_ADAPTIVE_WEIGHT="true"
USE_DYNAMIC_FILTER="true"

SEQS=(00 01 02 03 04 05 06 07 08 09 10)

#############################
# Prepare folders
#############################
mkdir -p "$OUT_DIR"
mkdir -p "$TUM_ROOT"

echo "[INFO] Logs will be saved to: $OUT_DIR"

#############################
# Main loop
#############################
for SEQ in "${SEQS[@]}"; do
  BAG_FILE="${DATASET_DIR}/${SEQ}.bag"
  if [[ ! -f "$BAG_FILE" ]]; then
    echo "[WARN] Bag not found: $BAG_FILE  -> skip seq $SEQ"
    continue
  fi

  LOG_FILE="${OUT_DIR}/kitti_${SEQ}.txt"
  TUM_PATH="${TUM_ROOT}/kitti_${SEQ}.txt"

  echo "============================================================"
  echo "[INFO] Start seq=${SEQ}"
  echo "       log=${LOG_FILE}"
  echo "       tum=${TUM_PATH}"
  echo "============================================================"

  # Start roslaunch and redirect all console output into TXT
  roslaunch "$LAUNCH_PKG" "$LAUNCH_FILE" \
    seq:="${SEQ}" \
    use_adaptive_weight:="${USE_ADAPTIVE_WEIGHT}" \
    use_dynamic_filter:="${USE_DYNAMIC_FILTER}" \
    tum_path:="${TUM_PATH}" \
    > "$LOG_FILE" 2>&1 &

  LAUNCH_PID=$!

  # Wait a bit for ROS to come up
  sleep 5

  # Wait for /rosbag_play to appear
  echo "[INFO][seq ${SEQ}] Waiting for /rosbag_play..."
  while true; do
    if rosnode list 2>/dev/null | grep -q "/rosbag_play"; then
      echo "[INFO][seq ${SEQ}] /rosbag_play is running."
      break
    fi
    if ! ps -p "$LAUNCH_PID" >/dev/null 2>&1; then
      echo "[ERROR][seq ${SEQ}] roslaunch exited unexpectedly. Check: $LOG_FILE"
      break
    fi
    sleep 1
  done

  # Wait for /rosbag_play to finish
  echo "[INFO][seq ${SEQ}] Waiting for /rosbag_play to finish..."
  while true; do
    if rosnode list 2>/dev/null | grep -q "/rosbag_play"; then
      sleep 2
    else
      echo "[INFO][seq ${SEQ}] /rosbag_play finished."
      break
    fi
  done

  # Stop roslaunch gracefully
  echo "[INFO][seq ${SEQ}] Stopping roslaunch..."
  if ps -p "$LAUNCH_PID" >/dev/null 2>&1; then
    kill -SIGINT "$LAUNCH_PID" || true
    wait "$LAUNCH_PID" || true
  fi

  echo "[INFO][seq ${SEQ}] Done. Log saved: $LOG_FILE"
  echo
  sleep 2
done

echo "============================================================"
echo "[INFO] All done. 11 logs are in: $OUT_DIR"
echo "============================================================"
