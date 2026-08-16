#!/bin/bash
# 全量真实重跑 L1-L5（与正式提交同管线）+ 官方评分。
# 前置: export OPENAI_API_KEY=<GLM key>; 系统包 libosmesa6-dev/xvfb 已装, Xvfb :99 已启动。
# 用法: bash rerun_all.sh [起始关卡idx 0-4]   (默认从0开始, 可断点续跑)
set -u
cd "$(dirname "$0")"

export DISPLAY=:99 MUJOCO_GL=osmesa GATE_OLLAMA="true"
export LD_LIBRARY_PATH="/etc/dsw/runtime/dynamic_libs/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="src:robosuite/robosuite:robomimic:."
export OPENAI_BASE_URL="${OPENAI_BASE_URL:-https://open.bigmodel.cn/api/paas/v4}"
export OPENAI_MODEL="${OPENAI_MODEL:-glm-5.2}"

if [ -z "${OPENAI_API_KEY:-}" ]; then
  echo "ERROR: OPENAI_API_KEY 未设置 (GLM token, 重跑必需)" >&2
  exit 2
fi

ENVS=(FactorySorting1_3FO3ERFHISEM FactorySorting3_3FO3ERRPH7X9 FactorySorting5_3FO3ERTPXEUT FactorySorting7_3FO3ERFKY9RN FactorySorting9_3FO3ERT2C5FP)
TASKS=(
'For this task, you need to transport a blue, hollow plastic box. Please move it from the starting point "Pick Station 2" to the destination "Place Station 3". Please follow the Standard Operating Procedure (SOP).'
'Current Task Material Information:
Material Name: Green-rimmed storage bin
Starting Location: Pick Station 1
Target Location: Place Station 3
Quantity to Transport: 1'
'Please follow the SOP. The object is a blue material transfer bin. The Pick Station is Pick Station 1, and the Place Station is Place Station 2.'
'Please strictly adhere to the Standard Operating Procedure (SOP) for this task. The object to be handled is a blue, hollow plastic box. The Pick Station is designated as Pick Station 5, and the Place Station is designated as Place Station 2.'
'Move the three white-rimmed storage bins from Pick Station 6 to Place Station 1.'
)

START=${1:-0}
SUMMARY=()
for IDX in $(seq "$START" 4); do
  ENV=${ENVS[$IDX]}
  TS=$(date +%Y%m%d_%H%M%S)
  echo "==================== [L$((IDX+1)) idx=$IDX env=$ENV ts=$TS] ===================="
  .venv/bin/python -m robot_agent.task_subprocess_runner \
    --task "${TASKS[$IDX]}" --task-index "$IDX" --timestamp "$TS" \
    --result-json "recordings/$ENV/result_${TS}.json" --app-dir "."
  RC=$?
  echo "[L$((IDX+1))] runner exit=$RC"
  TRAJ=$(ls -t recordings/$ENV/trajectory_${TS}_*.json 2>/dev/null | head -1)
  if [ -z "$TRAJ" ]; then
    echo "[L$((IDX+1))] ✗ 无轨迹产出"
    SUMMARY+=("L$((IDX+1)): 无轨迹 (rc=$RC)")
    continue
  fi
  .venv/bin/python score_dev.py "$TRAJ" --task-index "$IDX" --save | tail -3
  SUMMARY+=("L$((IDX+1)): $(basename $TRAJ)")
  sleep 2
done

echo
echo "==================== 重跑汇总 ===================="
for s in "${SUMMARY[@]}"; do echo "  $s"; done
