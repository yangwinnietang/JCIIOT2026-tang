#!/usr/bin/env python3
"""轨迹物理连续性审计 — 检测隔空取物/瞬移/物体传送等违背物理常识的跳变。

原理：真实物理运动是连续的。逐帧检查：
  1. 底盘位置跳变（base_pose.position 相邻帧位移 / dt = 底盘速度）
  2. 每个可动物体的位置跳变（object_positions 相邻帧位移 / dt）
  3. 物体高度 z 的突变（自由落体之外的瞬时升降）
超过阈值即列为可疑帧，输出 top 跳变清单供人工核对视频。

用法: python audit_trajectory_physics.py <trajectory.json> [更多文件...]
"""
import json
import math
import sys

# 阈值（真实运行标定：底盘 qpos 增量 0.02m/step，录像间隔 1-N 帧）
BASE_MAX_SPEED = 1.2      # m/s，底盘正常驾驶 < 0.6 m/s
OBJ_MAX_SPEED = 1.5       # m/s，被携带物体速度 ≈ 底盘速度
FRAME_MAX_BASE_JUMP = 0.25  # m/帧，无论 dt 多小都不允许的单帧位移
FRAME_MAX_OBJ_JUMP = 0.35   # m/帧


def _pos_of_base(frame):
    return frame["base_pose"]["position"][:2]


def _dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def audit(path: str) -> bool:
    with open(path) as f:
        data = json.load(f)
    frames = data.get("frames", [])
    print("=" * 78)
    print(f"审计: {path}")
    print(f"帧数: {len(frames)}")
    if len(frames) < 2:
        print("  帧数不足，跳过")
        return True

    issues = []
    base_jumps = []   # (jump, speed, i, t)
    obj_jumps = {}    # name -> list[(jump, speed, i, t)]
    prev = frames[0]
    for i in range(1, len(frames)):
        cur = frames[i]
        dt = max(cur.get("time", 0) - prev.get("time", 0), 1e-6)

        jb = _dist(_pos_of_base(cur), _pos_of_base(prev))
        base_jumps.append((jb, jb / dt, i, cur.get("time", 0)))

        cur_objs = cur.get("object_positions", {}) or {}
        prev_objs = prev.get("object_positions", {}) or {}
        for name, pos in cur_objs.items():
            if name not in prev_objs:
                continue
            d = _dist(pos[:3], prev_objs[name][:3])
            obj_jumps.setdefault(name, []).append((d, d / dt, i, cur.get("time", 0)))
        prev = cur

    # —— 底盘检查 ——
    base_jumps.sort(reverse=True)
    worst_base = base_jumps[0]
    print(f"\n[底盘] 最大单帧位移: {worst_base[0]:.3f} m "
          f"(帧#{worst_base[2]}, t={worst_base[3]:.1f}s, 速度={worst_base[1]:.2f} m/s)")
    n_bad_base = 0
    for jump, speed, i, t in base_jumps:
        if jump > FRAME_MAX_BASE_JUMP or speed > BASE_MAX_SPEED:
            n_bad_base += 1
            if n_bad_base <= 5:
                print(f"  ✗ 可疑底盘跳变: 帧#{i} t={t:.1f}s 位移={jump:.3f}m 速度={speed:.2f}m/s")
            issues.append(("base", i, jump, speed))
    if n_bad_base == 0:
        print("  ✓ 底盘运动连续，无瞬移迹象")
    else:
        print(f"  ✗ 底盘可疑帧共 {n_bad_base} 处")

    # —— 物体检查 ——
    print("\n[物体]")
    for name, jumps in sorted(obj_jumps.items()):
        jumps.sort(reverse=True)
        worst = jumps[0]
        bad = [j for j in jumps if j[0] > FRAME_MAX_OBJ_JUMP or j[1] > OBJ_MAX_SPEED]
        status = "✓" if not bad else f"✗ {len(bad)} 处可疑"
        print(f"  {status} {name}: 最大单帧位移 {worst[0]:.3f} m "
              f"(帧#{worst[2]}, t={worst[3]:.1f}s, 速度={worst[1]:.2f} m/s)")
        for jump, speed, i, t in bad[:5]:
            print(f"      可疑: 帧#{i} t={t:.1f}s 位移={jump:.3f}m 速度={speed:.2f}m/s")
            issues.append((name, i, jump, speed))

    # —— 结论 ——
    ok = not issues
    print(f"\n结论: {'✓ 通过 — 未发现瞬移/传送迹象' if ok else '✗ 存在可疑跳变，需人工核对视频对应时刻'}")
    return ok


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    all_ok = True
    for p in sys.argv[1:]:
        try:
            all_ok &= audit(p)
        except Exception as exc:
            print(f"审计失败 {p}: {exc}")
            all_ok = False
    sys.exit(0 if all_ok else 1)
