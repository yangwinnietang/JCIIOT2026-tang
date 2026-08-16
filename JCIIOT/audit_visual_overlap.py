#!/usr/bin/env python3
"""轨迹可视层 3D 重叠审计 — 检测机器人/持物的**可视几何**与场景**可视几何**的真实穿插。

与 audit_contacts.py 的区别：接触审计只能看到碰撞几何（contype>0）的接触；
本场景大量设备是纯可视网格（contype=0/conaffinity=0，factory_sorting.py 中
obj_type="visual"），物理上永远不产生接触，但视频里会发生肉眼可见的穿插。
本工具对每一帧用 mj_geomDistance（支持 mesh 凸包）计算有符号距离：
  dist < -threshold  → 真实可视穿插（视频可见缺陷）
  dist >= 0          → 有间隙（若视频中看似重叠则为投影错觉）

分类：
  R: 机器人本体（底盘/躯干/头/手臂连杆，夹爪指除外） vs 场景可视几何
  O: 被搬运物体 vs 场景可视几何
夹爪指(finger/fingertip/pad)不参与判定（抓取接触为正常操作）。

用法:
  python audit_visual_overlap.py <level> <trajectory.json> [--frames A:B] [--threshold 0.003] [--verbose-frames F1,F2]
"""
import argparse
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

LEVEL_ENV = {
    "L1": "FactorySorting1_3FO3ERFHISEM",
    "L2": "FactorySorting3_3FO3ERRPH7X9",
    "L3": "FactorySorting5_3FO3ERTPXEUT",
    "L4": "FactorySorting7_3FO3ERFKY9RN",
    "L5": "FactorySorting9_3FO3ERT2C5FP",
}
GRIPPER_TOKENS = ("finger", "fingertip", "fingerpad", "gripper0_finger")
FLOOR_TOKENS = ("floor", "ground", "wall", "ceiling")


def _geom_name(sim, gid):
    return sim.model.geom_id2name(gid) or f"geom#{gid}"


def _body_name(sim, bid):
    return sim.model.body_id2name(bid) or f"body#{bid}"


def build_geom_sets(sim, obj_names):
    """返回 (robot_geoms, object_geoms, scene_geoms)；全部只含**可见**(group==1)几何。

    group 0 = 碰撞壳（视频不渲染）；group 3 = support 辅助（不可见）；
    group 1 = 可视网格（机器人 *_vis、物体 *_visual、场景 usd_*/桌面等）。
    """
    model = sim.model
    robot_ids, obj_ids, scene_ids = [], [], []
    for gid in range(model.ngeom):
        if int(model.geom_group[gid]) != 1:
            continue
        gname = _geom_name(sim, gid)
        bname = _body_name(sim, model.geom_bodyid[gid])
        is_robot = (gname.startswith("robot0_") or gname.startswith("gripper0_")
                    or bname.startswith("robot0") or bname.startswith("gripper0"))
        if is_robot:
            if not any(tok in gname for tok in GRIPPER_TOKENS):
                robot_ids.append(gid)
            continue
        root = next((o for o in obj_names if gname == o or gname.startswith(o + "_")), None)
        if root is not None:
            obj_ids.append(gid)
            continue
        alpha = float(model.geom_rgba[gid][3])
        if alpha < 0.05:
            continue
        lname = gname.lower()
        if any(tok in lname for tok in FLOOR_TOKENS):
            continue
        scene_ids.append(gid)
    return robot_ids, obj_ids, scene_ids


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("level", choices=list(LEVEL_ENV))
    ap.add_argument("trajectory")
    ap.add_argument("--frames", default=None, help="A:B 只审计 [A,B) 帧")
    ap.add_argument("--threshold", type=float, default=0.003, help="穿插深度阈值(米)")
    ap.add_argument("--near", type=float, default=0.02, help="verbose 帧的近距报告上限(米)")
    ap.add_argument("--verbose-frames", default=None, help="逗号分隔帧号，打印全部近距对")
    args = ap.parse_args()

    import mujoco
    import robot_agent.skills.pick_up  # noqa: F401  (应用运行时补丁)
    from robot_agent.environments import RobosuiteBackend
    from robosuite.environments.factory_sorting.load_factory_sorting_evalization import (
        get_base_world_pose, _find_base_joint_addrs,
    )

    level = args.level
    data = json.loads(Path(args.trajectory).read_text(encoding="utf-8"))
    frames = data["frames"]
    obj_names = list(data.get("object_names", []))
    moved = sorted({e.get("object_name") for e in data.get("events", [])
                    if e.get("name") == "grasp_end" and e.get("object_name")})
    lo, hi = 0, len(frames)
    if args.frames:
        a, b = args.frames.split(":")
        lo, hi = int(a), int(b)
    verbose_frames = set()
    if args.verbose_frames:
        verbose_frames = {int(x) for x in args.verbose_frames.split(",")}
    print(f"[{level}] 帧 {lo}..{hi-1} / 共 {len(frames)}, 搬运物体: {moved}", flush=True)

    backend = RobosuiteBackend(env_name=LEVEL_ENV[level], camera="birdview",
                               drive_mode="direct", headless=True)
    backend.reset()
    env = backend.env
    sim = env.sim
    model = sim.model

    # 物体关节
    obj_joints = {}
    for name in obj_names:
        for suffix in ("_joint0", "_free"):
            jn = f"{name}{suffix}"
            try:
                model.get_joint_qpos_addr(jn)
                obj_joints[name] = jn
                break
            except Exception:
                continue

    robot_g, obj_g, scene_g = build_geom_sets(sim, obj_names)
    print(f"几何集合: 机器人 {len(robot_g)}, 物体 {len(obj_g)}, 场景可视 {len(scene_g)}", flush=True)
    scene_g_arr = np.array(scene_g, dtype=int)
    qadr = model.jnt_qposadr
    fwd, sid, yad = _find_base_joint_addrs(sim)

    # mj_geomDistance 需要底层 mujoco 对象（robosuite 包装类会被 pybind 拒绝）
    mm = getattr(sim.model, "_model", sim.model)
    md = getattr(sim.data, "_data", sim.data)

    # 距离引擎自检：超大 distmax 下查询机器人底座 vs 任意场景几何，
    # 若仍返回 distmax 说明调用链断裂（如包装类被拒），立即中止而不是静默失明。
    fromto0 = np.zeros(6)
    selftest = mujoco.mj_geomDistance(mm, md, robot_g[0], scene_g[0], 1e6, fromto0)
    if selftest >= 1e6:
        print("FATAL: mj_geomDistance 自检失败（返回 distmax），审计无效", flush=True)
        return 2
    print(f"距离引擎自检 OK (sample dist={selftest:.3f}m)", flush=True)

    thr = -abs(args.threshold)
    events = []  # (fi, time, cat, g1name, g2name, dist)
    verbose_dump = defaultdict(list)
    dist_fails = 0
    min_seen = {}  # cat -> (dist, fi, n1, n2)

    def set_base(frame):
        bp = frame["base_pose"]["position"]
        bq = frame["base_pose"]["orientation_xyzw"]
        yaw = float(np.arctan2(2 * bq[3] * bq[2], 1 - 2 * bq[2] * bq[2]))
        cur_xy, cur_yaw = get_base_world_pose(env)
        eps = 1e-4
        q0 = float(sim.data.qpos[qadr[fwd]])
        sim.data.qpos[qadr[fwd]] = q0 + eps
        sim.forward()
        df = (np.array(get_base_world_pose(env)[0]) - np.array(cur_xy)) / eps
        sim.data.qpos[qadr[fwd]] = q0
        q0 = float(sim.data.qpos[qadr[sid]])
        sim.data.qpos[qadr[sid]] = q0 + eps
        sim.forward()
        ds = (np.array(get_base_world_pose(env)[0]) - np.array(cur_xy)) / eps
        sim.data.qpos[qadr[sid]] = q0
        sim.forward()
        J = np.column_stack([df, ds])
        dq = np.linalg.inv(J) @ (np.array(bp[:2]) - np.array(cur_xy))
        sim.data.qpos[qadr[fwd]] += dq[0]
        sim.data.qpos[qadr[sid]] += dq[1]
        sim.data.qpos[qadr[yad]] += (yaw - cur_yaw + np.pi) % (2 * np.pi) - np.pi

    def scan_pairs(fi, t, mover_ids, cat):
        if not mover_ids or len(scene_g) == 0:
            return
        mpos = sim.data.geom_xpos[np.array(mover_ids, dtype=int)]      # (M,3)
        mr = model.geom_rbound[np.array(mover_ids, dtype=int)]         # (M,)
        spos = sim.data.geom_xpos[scene_g_arr]                          # (S,3)
        sr = model.geom_rbound[scene_g_arr]                             # (S,)
        # 包围球预筛
        d = (mpos[:, None, :] - spos[None, :, :])
        dist = np.sqrt((d ** 2).sum(-1)) - mr[:, None] - sr[None, :]
        cand = np.argwhere(dist < args.near)
        nonlocal_fail = 0
        for mi, si in cand:
            g1 = mover_ids[mi]
            g2 = int(scene_g_arr[si])
            fromto = np.zeros(6)
            try:
                dd = mujoco.mj_geomDistance(mm, md, g1, g2, args.near, fromto)
            except Exception:
                nonlocal_fail += 1
                continue
            rec = (fi, t, cat, _geom_name(sim, g1), _geom_name(sim, g2), float(dd))
            if fi in verbose_frames:
                verbose_dump[fi].append(rec)
            k = cat
            if k not in min_seen or dd < min_seen[k][0]:
                min_seen[k] = (float(dd), fi, rec[3], rec[4])
            if dd < thr:
                events.append(rec)
        return nonlocal_fail

    for fi in range(lo, hi):
        frame = frames[fi]
        set_base(frame)
        for jn, val in frame.get("joint_positions", {}).items():
            try:
                sim.data.set_joint_qpos(jn, val)
            except Exception:
                pass
        for oname, vals in frame.get("object_positions", {}).items():
            jn = obj_joints.get(oname)
            if jn:
                try:
                    sim.data.set_joint_qpos(jn, list(vals))
                except Exception:
                    pass
        sim.forward()
        t = round(float(frame.get("time", 0)), 2)
        dist_fails += scan_pairs(fi, t, robot_g, "R") or 0
        dist_fails += scan_pairs(fi, t, obj_g, "O") or 0
        if fi % 500 == 0:
            print(f"  .. 帧 {fi} (t={t}s), 累计穿插 {len(events)}", flush=True)

    backend.close()

    if verbose_dump:
        print("\n== 指定帧全部近距对（dist < %.0fmm，含正间隙）==" % (args.near * 1000))
        for fi in sorted(verbose_dump):
            print(f" 帧#{fi}:")
            for _, tt, cat, n1, n2, dd in sorted(verbose_dump[fi], key=lambda r: r[5]):
                print(f"   [{cat}] {n1}  <->  {n2}   dist={dd*1000:+.1f}mm")

    print(f"\n距离计算失败数: {dist_fails}（>0 说明存在盲区，结果不可信）")
    print("== 全程最小间隙（每类）==")
    for cat, (dd, fi, n1, n2) in sorted(min_seen.items()):
        print(f"  [{cat}] 最小 {dd*1000:+.1f}mm @帧{fi}: {n1} <-> {n2}")

    print(f"\n== 可视穿插事件（dist < {thr*1000:.1f}mm，仅 group==1 可见几何）==")
    if not events:
        if dist_fails > 0:
            print("  ? 无穿插记录，但存在距离计算失败——结论存疑")
            return 2
        print("  ✓ 全程无真实可视穿插（视频中的'重叠'均为投影错觉/误认）")
        return 0
    # 事件全量落盘（供三角面复核定位）
    out_json = Path(f"/tmp/vo_events_{level}.json")
    agg = {}
    for fi, tt, cat, n1, n2, dd in events:
        k = (cat, n1, n2)
        a = agg.setdefault(k, {"count": 0, "worst": 0.0, "worst_frame": 0,
                               "first": 10**9, "last": -1})
        a["count"] += 1
        if dd < a["worst"]:
            a["worst"] = dd
            a["worst_frame"] = fi
        a["first"] = min(a["first"], fi)
        a["last"] = max(a["last"], fi)
    out_json.write_text(json.dumps(
        [{"cat": k[0], "g1": k[1], "g2": k[2], **v} for k, v in agg.items()],
        ensure_ascii=False, indent=1))
    n_frames_total = hi - lo
    print(f"  全量事件已存 {out_json}")
    for cat in ("R", "O"):
        print(f"  -- [{cat}] --")
        items = [(k, v) for k, v in agg.items() if k[0] == cat]
        for (c, n1, n2), v in sorted(items, key=lambda kv: kv[1]["first"]):
            inherent = " [帧0固有]" if v["first"] <= lo + 1 and v["count"] > 0.9 * n_frames_total else ""
            print(f"  ✗ {n1} <-> {n2}: {v['count']} 帧, 最深 {v['worst']*1000:.1f}mm"
                  f" @帧{v['worst_frame']}, 区间 {v['first']}..{v['last']}{inherent}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
