#!/usr/bin/env bash
set -euo pipefail

#############################
# Config (modify if needed)
#############################
LAUNCH_PKG="vtd_lo"          # 你的launch所在包名：例如 vtd_lo
LAUNCH_FILE="kitti.launch"   # 你的launch文件名：kitti.launch

DATASET_DIR="$HOME/DATASET/KITTI"   # 00.bag ~ 10.bag 所在目录（仅用于检查）
OUT_DIR="$HOME/VTD-LO/result/vtd_lo_logs_kitti"   # 11个TXT输出目录

# 可选：给每次运行指定TUM轨迹保存目录（如果你的launch/节点需要）
TUM_ROOT="$HOME/VTD-LO/result/tum"

# 可选：消融开关（保持launch默认true也行）
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
