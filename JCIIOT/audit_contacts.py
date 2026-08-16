#!/usr/bin/env python3
"""轨迹 3D 接触审计 — 检测物体与机器人非夹爪部位/场景之间的穿透（重叠）。

逐帧回放轨迹，用 MuJoCo 接触检测计算：
  1. 任务物体 vs 机器人本体（躯干/底盘/手臂连杆，夹爪指垫/指尖除外——那是正常抓取接触）
  2. 任务物体 vs 场景元素（桌子/地面/产线/围栏等）
穿透深度 < -5mm 判定为可疑重叠（静止接触 dist≈0 属正常）。

用法: python audit_contacts.py <level> <trajectory.json>
"""
import json
import sys
from pathlib import Path

import numpy as np

LEVEL_ENV = {
    "L1": "FactorySorting1_3FO3ERFHISEM",
    "L2": "FactorySorting3_3FO3ERRPH7X9",
    "L3": "FactorySorting5_3FO3ERTPXEUT",
    "L4": "FactorySorting7_3FO3ERFKY9RN",
    "L5": "FactorySorting9_3FO3ERT2C5FP",
}
PEN_TOLERANCE = -0.005  # 5mm
# 正常抓取接触（夹爪指垫/指尖）白名单
GRIPPER_OK = ("fingerpad", "fingertip")


def main(level: str, traj_path: str) -> bool:
    import robot_agent.skills.pick_up  # noqa: F401  (应用运行时补丁: _find_base_joint_addrs 等)
    from robot_agent.environments import RobosuiteBackend

    env_name = LEVEL_ENV[level]
    data = json.loads(Path(traj_path).read_text(encoding="utf-8"))
    frames = data["frames"]
    obj_names = list(data.get("object_names", []))
    # 被搬运的物体（grasp_end 事件中出现过的）
    moved = sorted({e.get("object_name") for e in data.get("events", [])
                    if e.get("name") == "grasp_end" and e.get("object_name")})
    print(f"[{level}] {len(frames)} 帧, 搬运物体: {moved}")
    all_objs = list(obj_names)

    backend = RobosuiteBackend(env_name=env_name, camera="birdview", drive_mode="direct")
    backend.reset()
    env = backend.env
    sim = env.sim

    # 预解析：物体关节地址、机器人 geom 集合
    obj_joints: dict[str, str] = {}
    for name in obj_names:
        for suffix in ("_joint0", "_free"):
            jn = f"{name}{suffix}"
            try:
                sim.model.get_joint_qpos_addr(jn)
                obj_joints[name] = jn
                break
            except Exception:
                continue

    base_joint_names = [n for n in sim.model.joint_names if "mobilebase" in n]

    robot_geom_ids = set()
    for gid in range(sim.model.ngeom):
        gname = sim.model.geom_id2name(gid) or ""
        if gname.startswith("robot0_") or gname.startswith("gripper0_"):
            robot_geom_ids.add(gid)

    def is_gripper_ok(gname: str) -> bool:
        return any(tok in gname for tok in GRIPPER_OK)

    flagged = []
    n_frames = len(frames)
    for fi, frame in enumerate(frames):
        # 设置底盘
        bp = frame["base_pose"]["position"]
        bq = frame["base_pose"]["orientation_xyzw"]
        yaw = float(np.arctan2(2 * bq[3] * bq[2], 1 - 2 * bq[2] * bq[2]))
        # 用世界位姿→关节值（mobilebase qpos 为出生点相对量）
        # 通过当前 site 读数求增量
        from robosuite.environments.factory_sorting.load_factory_sorting_evalization import (
            get_base_world_pose, _find_base_joint_addrs,
        )
        cur_xy, cur_yaw = get_base_world_pose(env)
        fwd, sid, yad = _find_base_joint_addrs(sim)
        qadr = sim.model.jnt_qposadr
        # 世界增量 → 关节增量（雅可比数值法，与 _drive_base_to 相同）
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
        # 关节
        for jn, val in frame.get("joint_positions", {}).items():
            try:
                sim.data.set_joint_qpos(jn, val)
            except Exception:
                pass
        # 物体
        for oname, vals in frame.get("object_positions", {}).items():
            jn = obj_joints.get(oname)
            if jn:
                try:
                    sim.data.set_joint_qpos(jn, list(vals))
                except Exception:
                    pass
        sim.forward()

        # 接触检查: (a) 任意物体 vs 机器人非夹爪部位 (b) 物体 vs 其他物体
        def _obj_root(gname):
            for o in all_objs:
                if gname == o or gname.startswith(o + "_"):
                    return o
            return None
        for ci in range(sim.data.ncon):
            c = sim.data.contact[ci]
            if c.dist >= PEN_TOLERANCE:
                continue
            g1 = sim.model.geom_id2name(c.geom1) or ""
            g2 = sim.model.geom_id2name(c.geom2) or ""
            r1, r2 = _obj_root(g1), _obj_root(g2)
            if r1 is None and r2 is None:
                continue
            if r1 is not None and r1 == r2:
                continue  # 同物体自身 geom(含支撑与壁板) = 正常静置
            if r1 is not None and r2 is not None:
                # 物体-物体穿透(不同物体)
                flagged.append((fi, round(float(frame.get("time", 0)), 1),
                                f"{r1}<->{r2}", "obj-obj", round(float(c.dist), 4)))
                continue
            obj_hit = r1 or r2
            other = g2 if r1 else g1
            if is_gripper_ok(other):
                continue  # 夹爪指垫/指尖 = 正常抓取接触
            flagged.append((fi, round(float(frame.get("time", 0)), 1), obj_hit, other, round(float(c.dist), 4)))

    # 汇总（按物体/对方去重计数）
    print(f"穿透接触 (dist < {PEN_TOLERANCE*1000:.0f}mm, 不含夹爪指垫):")
    if not flagged:
        print("  ✓ 全程无可疑穿透/重叠")
        return True
    from collections import Counter
    cnt = Counter((o, g) for _, _, o, g, _ in flagged)
    for (o, g), n in cnt.most_common(20):
        worst = min(d for _, _, oo, gg, d in flagged if oo == o and gg == g)
        first = next(f for f, _, oo, gg, _ in flagged if oo == o and gg == g)
        print(f"  ✗ {o} vs {g}: {n} 次, 最深 {worst*1000:.1f}mm, 首见帧#{first}")
    return False


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    ok = main(sys.argv[1], sys.argv[2])
    sys.exit(0 if ok else 1)
