#!/bin/bash
# 三角面级精确复核全部关键凸包事件 → /tmp/tri_verify.log
set -u
cd "$(dirname "$0")"
export DISPLAY=:99 MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
export LD_LIBRARY_PATH="/etc/dsw/runtime/dynamic_libs/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="src:robosuite/robosuite:robomimic:."

T1="recordings/FactorySorting1_3FO3ERFHISEM/trajectory_20260815_234304_OK.json"
T2="recordings/FactorySorting3_3FO3ERRPH7X9/trajectory_20260816_010531_OK.json"
T3="recordings/FactorySorting5_3FO3ERTPXEUT/trajectory_20260816_010915_OK.json"
T4="recordings/FactorySorting7_3FO3ERFKY9RN/trajectory_20260816_011313_OK.json"
T5="recordings/FactorySorting9_3FO3ERT2C5FP/trajectory_20260816_012725_OK.json"

run() { echo "===== $* ====="; .venv/bin/python verify_visual_triangle.py "$@" 2>&1 | grep -vE "robosuite|SyntaxWarning|/\[_\]"; }

# 机器人 vs 设备（运输/站位段）
run L1 "$T1" --frame 1192 --pairs "robot0_g1_vis:usd_0436" "robot0_g0_vis:usd_0432" "robot0_g14_vis:usd_0432" "robot0_g13_vis:usd_0432" "robot0_g12_vis:usd_0432"
run L4 "$T4" --frame 1157 --pairs "robot0_g14_vis:usd_0257" "robot0_g12_vis:usd_0257" "robot0_g13_vis:usd_0257" "robot0_g26_vis:usd_0257"
run L5 "$T5" --frame 3103 --pairs "robot0_g1_vis:usd_0373" "robot0_g0_vis:usd_0373" "robot0_g14_vis:usd_0369" "robot0_g2_vis:usd_0373"
run L5 "$T5" --frame 4883 --pairs "robot0_g12_vis:usd_0369" "robot0_g13_vis:usd_0369"
# 放置后物体 vs 目标站机架
run L1 "$T1" --frame 1966 --pairs "line_5_container_h01_near_visual:usd_0393" "line_5_container_h01_near_visual:usd_0412"
run L2 "$T2" --frame 1723 --pairs "green_tote_b01_upper_visual:usd_0412"
run L3 "$T3" --frame 1895 --pairs "blue_tote_b01_far_right_visual:usd_0150"
run L4 "$T4" --frame 2443 --pairs "blue_container_h01_back_upper_visual:usd_0441"
run L4 "$T4" --frame 2652 --pairs "blue_container_h01_back_upper_visual:usd_0155"
run L5 "$T5" --frame 2477 --pairs "white_tote_b01_left_center_visual:usd_0242"
run L5 "$T5" --frame 4254 --pairs "white_tote_b01_left_front_visual:usd_0242"
# 抓取段夹爪伸入源站货架（预期为必要操作）
run L1 "$T1" --frame 655 --pairs "gripper0_right_right_inner_knuckle_visual:usd_0088"
echo "ALL DONE"
