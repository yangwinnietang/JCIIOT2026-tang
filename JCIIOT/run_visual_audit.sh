#!/bin/bash
# 全关卡可视重叠审计（纯物理回放，无需渲染）
set -u
cd "$(dirname "$0")"
export DISPLAY=:99 MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
export LD_LIBRARY_PATH="/etc/dsw/runtime/dynamic_libs/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="src:robosuite/robosuite:robomimic:."

declare -A TRAJ=(
  [L1]="recordings/FactorySorting1_3FO3ERFHISEM/trajectory_20260816_111213_OK.json"
  [L2]="recordings/FactorySorting3_3FO3ERRPH7X9/trajectory_20260816_111600_OK.json"
  [L3]="recordings/FactorySorting5_3FO3ERTPXEUT/trajectory_20260816_111938_OK.json"
  [L4]="recordings/FactorySorting7_3FO3ERFKY9RN/trajectory_20260816_112331_OK.json"
  [L5]="recordings/FactorySorting9_3FO3ERT2C5FP/trajectory_20260816_112911_OK.json"
)

for lv in L1 L2 L3 L4 L5; do
  .venv/bin/python audit_visual_overlap.py "$lv" "${TRAJ[$lv]}" \
    --threshold 0.003 --near 0.02 > "/tmp/vo_${lv}.log" 2>&1 &
done
wait
echo "===== 汇总 ====="
for lv in L1 L2 L3 L4 L5; do
  echo "--- $lv ---"
  grep -E "^\[|几何集合|✓|✗" "/tmp/vo_${lv}.log" | head -30
done
