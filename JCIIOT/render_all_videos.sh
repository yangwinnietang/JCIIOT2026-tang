#!/bin/bash
# 渲染全部 15 个三视角视频 (640x480, step2, 鸟瞰去视差 2.0) 到 videos/。
# 用法: bash render_all_videos.sh [每批并行数, 默认1=串行(防GL争抢花屏)]
set -u
cd "$(dirname "$0")"
export DISPLAY=:99 MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
export LD_LIBRARY_PATH="/etc/dsw/runtime/dynamic_libs/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="src:robosuite/robosuite:."
PAR=${1:-1}

declare -A TRAJ=(
  [L1]="recordings/FactorySorting1_3FO3ERFHISEM/trajectory_20260816_111213_OK.json"
  [L2]="recordings/FactorySorting3_3FO3ERRPH7X9/trajectory_20260816_111600_OK.json"
  [L3]="recordings/FactorySorting5_3FO3ERTPXEUT/trajectory_20260816_111938_OK.json"
  [L4]="recordings/FactorySorting7_3FO3ERFKY9RN/trajectory_20260816_112331_OK.json"
  [L5]="recordings/FactorySorting9_3FO3ERT2C5FP/trajectory_20260816_112911_OK.json"
)
declare -A OUTDIR=(
  [L1]="recordings/FactorySorting1_3FO3ERFHISEM"
  [L2]="recordings/FactorySorting3_3FO3ERRPH7X9"
  [L3]="recordings/FactorySorting5_3FO3ERTPXEUT"
  [L4]="recordings/FactorySorting7_3FO3ERFKY9RN"
  [L5]="recordings/FactorySorting9_3FO3ERT2C5FP"
)

mkdir -p ../videos
pids=()
for lv in L1 L2 L3 L4 L5; do
  (
    .venv/bin/python3 replay_to_video.py --trajectory "${TRAJ[$lv]}" --camera all \
      --full --step 2 --width 640 --height 480 --birdview-flat 2.0 || echo "[$lv] 渲染失败"
    # 拷贝为规范命名
    base=$(basename "${TRAJ[$lv]}" .json)   # trajectory_2026..._OK
    rep=${base/trajectory_/replay_}
    cp -f "${OUTDIR[$lv]}/${rep}.mp4"                    "../videos/${lv}_birdview_鸟瞰.mp4" 2>/dev/null
    cp -f "${OUTDIR[$lv]}/${rep}_robot0_robotview.mp4"   "../videos/${lv}_机器人第一人称.mp4" 2>/dev/null
    cp -f "${OUTDIR[$lv]}/${rep}_follow.mp4"             "../videos/${lv}_follow_跟随.mp4" 2>/dev/null
    echo "[$lv] 完成"
  ) &
  pids+=($!)
  # 限流
  while [ "$(jobs -rp | wc -l)" -ge "$PAR" ]; do sleep 5; done
done
wait
echo "全部渲染完成，开始视频完整性验证:"
.venv/bin/python3 verify_videos.py --dir ../videos
echo "验证退出码: $?"
ls -la ../videos/