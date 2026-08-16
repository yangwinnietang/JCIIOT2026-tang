#!/bin/bash
# 全审计: 对每关最新 OK 轨迹依次运行 完整性/物理连续性/接触穿透/场景完整性 四套审计。
# 用法: bash run_all_audits.sh
set -u
cd "$(dirname "$0")"
export DISPLAY=:99 MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
export LD_LIBRARY_PATH="/etc/dsw/runtime/dynamic_libs/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="src:robosuite/robosuite:."

ENVS=(FactorySorting1_3FO3ERFHISEM FactorySorting3_3FO3ERRPH7X9 FactorySorting5_3FO3ERTPXEUT FactorySorting7_3FO3ERFKY9RN FactorySorting9_3FO3ERT2C5FP)
LVLS=(L1 L2 L3 L4 L5)
IDXS=(0 1 2 3 4)
TRAJS=()
FAIL=0

for i in 0 1 2 3 4; do
  T=$(ls -t recordings/${ENVS[$i]}/trajectory_*_OK.json 2>/dev/null | head -1)
  if [ -z "$T" ]; then echo "[${LVLS[$i]}] 无 OK 轨迹"; FAIL=1; continue; fi
  TRAJS+=("$T")
  echo "=== ${LVLS[$i]}: $(basename $T)"
done

echo
echo "########## 1/4 轨迹完整性 (verify_trajectories) ##########"
.venv/bin/python3 verify_trajectories.py || FAIL=1

echo
echo "########## 2/4 物理连续性 (audit_trajectory_physics) ##########"
.venv/bin/python3 audit_trajectory_physics.py "${TRAJS[@]}" || FAIL=1

echo
echo "########## 3/4 接触穿透 (audit_contacts, 全物体+物体-物体) ##########"
for i in 0 1 2 3 4; do
  [ -z "${TRAJS[$i]:-}" ] && continue
  .venv/bin/python3 audit_contacts.py "${LVLS[$i]}" "${TRAJS[$i]}" || FAIL=1
done

echo
echo "########## 4/4 场景完整性: 撞落/推行/放置扰动 (audit_scene_integrity) ##########"
.venv/bin/python3 audit_scene_integrity.py "${TRAJS[@]}" || FAIL=1

echo
if [ "$FAIL" = "0" ]; then
  echo "✓✓ 全审计通过"
else
  echo "✗✗ 存在未通过项 (见上)"
fi
exit $FAIL
