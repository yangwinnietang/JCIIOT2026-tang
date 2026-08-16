#!/usr/bin/env python3
"""可视壳护栏回放验证 — 对每条轨迹逐帧评估 _visual_shell_penetration。

目的：
  1. 护栏必须在已实锤的躯干穿插窗口触发（L1 f1174-1211 / L4 f1122-1165 / L5 三段）。
  2. 护栏不得在其他帧误触发（尤其抓取/放置站姿）——否则说明网格或阈值有误。

用法: python validate_visual_guard.py <level> <trajectory.json>
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

LEVEL_ENV = {
    "L1": "FactorySorting1_3FO3ERFHISEM",
    "L2": "FactorySorting3_3FO3ERRPH7X9",
    "L3": "FactorySorting5_3FO3ERTPXEUT",
    "L4": "FactorySorting7_3FO3ERFKY9RN",
    "L5": "FactorySorting9_3FO3ERT2C5FP",
}


def main(level: str, traj_path: str) -> int:
    import robot_agent.skills.pick_up  # noqa: F401
    from robot_agent.environments import RobosuiteBackend
    from robot_agent.skills._factory_physics_patch import (
        _visual_shell_penetration, _visual_shell_grid,
    )

    data = json.loads(Path(traj_path).read_text(encoding="utf-8"))
    frames = data["frames"]
    backend = RobosuiteBackend(env_name=LEVEL_ENV[level], camera="birdview",
                               drive_mode="direct", headless=True)
    backend.reset()
    env = backend.env

    grid, x0, y0, cell = _visual_shell_grid(env)
    print(f"[{level}] 危险网格: {grid.shape}, 占用格 {int(grid.sum())}", flush=True)

    hits = []
    for fi, frame in enumerate(frames):
        bp = frame["base_pose"]["position"]
        bq = frame["base_pose"]["orientation_xyzw"]
        yaw = float(np.arctan2(2 * bq[3] * bq[2], 1 - 2 * bq[2] * bq[2]))
        if _visual_shell_penetration(env, bp[:2], yaw):
            hits.append(fi)
        if fi % 1000 == 0:
            print(f"  .. 帧 {fi}/{len(frames)}, 累计触发 {len(hits)}", flush=True)
    backend.close()

    # 汇总触发区间
    print(f"触发帧数: {len(hits)} / {len(frames)}")
    if hits:
        spans = []
        s = p = hits[0]
        for h in hits[1:]:
            if h - p > 5:
                spans.append((s, p))
                s = h
            p = h
        spans.append((s, p))
        for s0, s1 in spans:
            print(f"  区间 {s0}..{s1} ({s1-s0+1} 帧)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
