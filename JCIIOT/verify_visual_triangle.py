#!/usr/bin/env python3
"""三角面级精确可视重叠验证 — 对指定帧的指定几何对给出真实表面距离。

mj_geomDistance 用的是凸包，对凹形机架网格会严重高估穿插（机器人站在
中空框架内也会被报成"穿透"）。本工具直接取网格三角面片，变换到世界系，
用 trimesh 做逐点-三角面精确距离 + 法向符号判定：
  signed < 0  → 真实表面穿插（视频中可见的"重叠"）
  signed >= 0 → 有间隙（凸包报告为凹形假象）

用法:
  python verify_visual_triangle.py <level> <trajectory.json> --frame 1154 \
      --pairs "robot0_g1_vis:usd_0261" "robot0_wheel_left_collision:usd_0257"
（--pairs 的两侧均为几何名子串，可一对多）
"""
import argparse
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
_lib = "/etc/dsw/runtime/dynamic_libs/lib"
if Path(_lib).exists():
    os.environ.setdefault("LD_LIBRARY_PATH", f"{_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}")
ROOT = Path(__file__).resolve().parent
for p in (str(ROOT), str(ROOT / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)
_RI = ROOT / "robosuite" / "robosuite"
if (_RI / "__init__.py").exists() and str(_RI) not in sys.path:
    sys.path.insert(0, str(_RI))

LEVEL_ENV = {
    "L1": "FactorySorting1_3FO3ERFHISEM",
    "L2": "FactorySorting3_3FO3ERRPH7X9",
    "L3": "FactorySorting5_3FO3ERTPXEUT",
    "L4": "FactorySorting7_3FO3ERFKY9RN",
    "L5": "FactorySorting9_3FO3ERT2C5FP",
}


def geom_world_trimesh(sim, gid):
    """返回该几何的世界系 trimesh.Trimesh（mesh 几何）或基本体素网格化。"""
    import numpy as np
    import trimesh
    m = sim.model
    gtype = m.geom_type[gid]
    xpos = np.asarray(sim.data.geom_xpos[gid], dtype=float)
    xmat = np.asarray(sim.data.geom_xmat[gid], dtype=float).reshape(3, 3)
    if gtype == 7:  # mesh
        mid = int(m.geom_dataid[gid])
        v0, vn = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
        f0, fn = int(m.mesh_faceadr[mid]), int(m.mesh_facenum[mid])
        verts = np.asarray(m.mesh_vert[v0:v0 + vn], dtype=float).reshape(-1, 3)
        faces = np.asarray(m.mesh_face[f0:f0 + fn], dtype=int).reshape(-1, 3)
        # mesh_vert 已是最终局部坐标（编译期完成缩放），直接用 geom 世界变换
        tm = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    else:
        size = np.asarray(m.geom_size[gid], dtype=float)
        if gtype == 2:    # sphere
            tm = trimesh.creation.icosphere(subdivisions=3, radius=size[0])
        elif gtype == 6:  # cylinder
            tm = trimesh.creation.cylinder(radius=size[0], height=2 * size[1], sections=32)
        elif gtype == 5:  # capsule
            tm = trimesh.creation.capsule(radius=size[0], height=2 * size[1])
        elif gtype == 0:  # plane -> 大盒近似（不用于本审计）
            tm = trimesh.creation.box(extents=(20, 20, 0.01))
        else:             # box 等
            tm = trimesh.creation.box(extents=2 * size[:3])
    T = np.eye(4)
    T[:3, :3] = xmat
    T[:3, 3] = xpos
    tm.apply_transform(T)
    return tm


def signed_surface_distance(tm_from, tm_to, n_samples=4000):
    """tm_from 表面采样点到 tm_to 三角面的**无符号**最近距离 + 射线奇偶包含判定。

    返回 (min_abs_dist, n_contained, n_total, contained_max_depth)。
    contained_max_depth = 被包含点到对方表面的最大距离（真实穿插深度的下界）。
    巨型网格先按对方包围盒+0.6m 裁剪（整机架网格数十万三角面，全量会 OOM）。
    """
    import numpy as np
    import trimesh

    def _crop(tm, other, margin=0.6):
        lo = other.bounds[0] - margin
        hi = other.bounds[1] + margin
        cent = tm.triangles.mean(axis=1)
        mask = np.all((cent >= lo) & (cent <= hi), axis=1)
        if mask.all():
            return tm
        sub = tm.submesh([np.nonzero(mask)[0]], append=False)
        return sub[0] if isinstance(sub, list) else sub

    tm_from_c = _crop(tm_from, tm_to)
    tm_to_c = _crop(tm_to, tm_from)
    pts, _ = trimesh.sample.sample_surface(tm_from_c, min(n_samples, max(500, len(tm_from_c.faces) * 4)))
    pts = np.vstack([tm_from_c.vertices, pts])
    closest, dists, tri_ids = trimesh.proximity.closest_point(tm_to_c, pts)
    min_abs = float(dists.min())
    # 射线奇偶包含（在 2.0m 裁剪网格上判定，兼顾内存与局部正确性；失败则报 -1）
    sub = pts[:: max(1, len(pts) // 1500)]
    n_contained = 0
    contained_max_depth = 0.0
    try:
        tm_cont = _crop(tm_to, tm_from, margin=2.0)
        inside = tm_cont.contains(sub)
        n_contained = int(inside.sum())
        if n_contained:
            dsub = trimesh.proximity.closest_point(tm_to_c, sub)[1]
            contained_max_depth = float(dsub[inside].max())
    except Exception:
        n_contained = -1  # 不可用
    return min_abs, n_contained, len(sub), contained_max_depth


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("level", choices=list(LEVEL_ENV))
    ap.add_argument("trajectory")
    ap.add_argument("--frame", type=int, required=True)
    ap.add_argument("--pairs", nargs="+", required=True,
                    help="'子串A:子串B'，A 侧与 B 侧各自匹配的所有几何两两验证")
    ap.add_argument("--samples", type=int, default=4000)
    args = ap.parse_args()

    import numpy as np
    import robot_agent.skills.pick_up  # noqa: F401
    from robot_agent.environments import RobosuiteBackend
    from robot_agent.environments.robosuite_backend import (
        _set_base_xy_direct, _set_base_world_yaw_direct,
    )

    data = json.loads(Path(args.trajectory).read_text(encoding="utf-8"))
    frames = data["frames"]
    frame = frames[args.frame]

    backend = RobosuiteBackend(env_name=LEVEL_ENV[args.level], camera="birdview",
                               drive_mode="direct", headless=True)
    backend.reset()
    sim = backend.env.sim
    robot = backend.env.robots[0]

    obj_joints = {}
    for name in data.get("object_names", []):
        for suffix in ("_joint0", "_free"):
            jn = f"{name}{suffix}"
            try:
                sim.model.get_joint_qpos_addr(jn)
                obj_joints[name] = jn
                break
            except Exception:
                continue

    bp = frame["base_pose"]["position"]
    bq = frame["base_pose"]["orientation_xyzw"]
    yaw = float(np.arctan2(2 * bq[3] * bq[2], 1 - 2 * bq[2] * bq[2]))
    _set_base_world_yaw_direct(backend.env, robot, yaw)
    _set_base_xy_direct(backend.env, robot, np.array(bp[:2], dtype=float))
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
    t = frame.get("time", 0)
    print(f"[{args.level}] 帧{args.frame} t={t:.1f}s base=({bp[0]:.2f},{bp[1]:.2f})", flush=True)

    # 自检：机器人底座 vs 地板距离应≈0（贴地），验证世界系变换正确
    g_floor = sim.model.geom_name2id("floor")
    g_base = None
    for gid in range(sim.model.ngeom):
        gn = sim.model.geom_id2name(gid) or ""
        if gn == "robot0_g0_vis":
            g_base = gid
            break
    if g_floor is not None and g_base is not None:
        tm_b = geom_world_trimesh(sim, g_base)
        zmin = float(tm_b.vertices[:, 2].min())
        print(f"自检: 机器人底座网格最低点 z={zmin:.3f}m（应≈0 贴地）", flush=True)

    overall_bad = False
    for pair in args.pairs:
        tok_a, tok_b = pair.split(":")
        gids_a = [g for g in range(sim.model.ngeom)
                  if tok_a in (sim.model.geom_id2name(g) or "")]
        gids_b = [g for g in range(sim.model.ngeom)
                  if tok_b in (sim.model.geom_id2name(g) or "")]
        if not gids_a or not gids_b:
            print(f"  ! 几何名未匹配: {tok_a} ({len(gids_a)}) / {tok_b} ({len(gids_b)})")
            continue
        for ga in gids_a:
            tm_a = geom_world_trimesh(sim, ga)
            for gb in gids_b:
                tm_b = geom_world_trimesh(sim, gb)
                md, n_in, ntot, in_depth = signed_surface_distance(tm_a, tm_b, args.samples)
                md2, n_in2, _, in_depth2 = signed_surface_distance(tm_b, tm_a, args.samples)
                name_a = sim.model.geom_id2name(ga)
                name_b = sim.model.geom_id2name(gb)
                min_gap = min(md, md2)
                # 判定：任一侧有采样点被对方包含且深度>1mm → 真实穿插；
                # 无包含但最近面距<1mm → 表面贴合；否则有间隙。
                pen = max(in_depth if n_in > 0 else 0.0,
                          in_depth2 if n_in2 > 0 else 0.0)
                if pen > 0.001:
                    tag = "✗ 真实穿插"
                    overall_bad = True
                elif min_gap < 0.001:
                    tag = "~ 表面贴合"
                else:
                    tag = "✓ 有间隙"
                print(f"  {tag}  {name_a} <-> {name_b}\n"
                      f"       最近面距 {min_gap*1000:+.1f}mm; "
                      f"A⊂B 点数 {n_in}/{ntot} (最深 {in_depth*1000:.1f}mm), "
                      f"B⊂A 点数 {n_in2} (最深 {in_depth2*1000:.1f}mm)", flush=True)
    backend.close()
    return 1 if overall_bad else 0


if __name__ == "__main__":
    sys.exit(main())
