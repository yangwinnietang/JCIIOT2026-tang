#!/bin/bash
# 打包竞赛提交资料：代码 + 轨迹 + 文档 + 视频 + MANIFEST（含 MD5 校验）
# 用法: bash build_submission_package.sh
set -eu
cd "$(dirname "$0")"
J=JCIIOT
PKG=submission_package
rm -rf "$PKG" submission_package.zip
mkdir -p "$PKG"/{code,trajectories,docs,videos}

# ── 代码（team_submission 全量，已与主代码逐字节核对）──
cp -r "$J/team_submission/." "$PKG/code/"
# 技术报告 + 复现说明
cp "$J/TECHNICAL_REPORT.md" "$PKG/docs/"
cp README.md "$PKG/docs/" 2>/dev/null || true

# ── 轨迹与评分（v2 最终 100/100 版）──
declare -A TRAJ=(
  [L1]="FactorySorting1_3FO3ERFHISEM/trajectory_20260816_111213_OK.json"
  [L2]="FactorySorting3_3FO3ERRPH7X9/trajectory_20260816_111600_OK.json"
  [L3]="FactorySorting5_3FO3ERTPXEUT/trajectory_20260816_111938_OK.json"
  [L4]="FactorySorting7_3FO3ERFKY9RN/trajectory_20260816_112331_OK.json"
  [L5]="FactorySorting9_3FO3ERT2C5FP/trajectory_20260816_112911_OK.json"
)
for lv in L1 L2 L3 L4 L5; do
  rel="${TRAJ[$lv]}"
  ts=$(basename "$rel" .json | sed 's/trajectory_//; s/_OK//')
  env=$(dirname "$rel")
  mkdir -p "$PKG/trajectories/$lv"
  cp "$J/recordings/$rel" "$PKG/trajectories/$lv/"
  for kind in score result scene_ready; do
    f=$(ls "$J/recordings/$env/${kind}_${ts}"*.json 2>/dev/null | head -1 || true)
    [ -n "$f" ] && cp "$f" "$PKG/trajectories/$lv/"
  done
done

# ── 视频（15 个三视角，渲染自上述轨迹）──
cp videos/*.mp4 "$PKG/videos/"

# ── MANIFEST ──
{
  echo "# JCIIOT 2026 提交包清单 (SOP-Runner)"
  echo
  echo "生成时间: $(date '+%Y-%m-%d %H:%M:%S')"
  echo
  echo "## 成绩（官方评分器 score_dev.py 原样调用 app.py 评分逻辑）"
  echo
  echo "| 关卡 | 轨迹 | 得分 |"
  echo "|------|------|:----:|"
  echo "| L1 | trajectory_20260816_111213_OK.json | 10/10 |"
  echo "| L2 | trajectory_20260816_111600_OK.json | 15/15 |"
  echo "| L3 | trajectory_20260816_111938_OK.json | 20/20 |"
  echo "| L4 | trajectory_20260816_112331_OK.json | 25/25 |"
  echo "| L5 | trajectory_20260816_112911_OK.json | 30/30 |"
  echo "| **总分** | | **100/100，全程零碰撞** |"
  echo
  echo "## 目录结构"
  echo
  echo '- `code/` — 路径生成相关代码（skills 含运行时补丁与可视壳导航、workflows、knowledge、models、config.yaml）'
  echo '- `trajectories/` — 五关最终轨迹 + score/result/scene_ready 配套 JSON'
  echo '- `docs/` — 方案说明（框架/技术路线/创新性/复现命令/修改文件清单）'
  echo '- `videos/` — 五关三视角演示视频（鸟瞰/第一人称/跟随，渲染自上述轨迹）'
  echo
  echo "## 文件 MD5 校验"
  echo
  echo '```'
  (cd "$PKG" && find . -type f | sort | xargs md5sum)
  echo '```'
} > "$PKG/MANIFEST.md"

zip -qr submission_package.zip "$PKG"
echo "✓ 打包完成: submission_package.zip ($(du -h submission_package.zip | cut -f1))"
ls -la "$PKG"
