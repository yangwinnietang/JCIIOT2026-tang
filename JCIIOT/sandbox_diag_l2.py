#!/usr/bin/env python3
"""L2 接触诊断 — 抓取后沿旧撤离路径逐点 dump 与 lower tote 的接触对。"""
import json
import math
import os
import sys

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
_lp = "/etc/dsw/runtime/dynamic_libs/lib"
if os.path.exists(_lp):
    os.environ["LD_LIBRARY_PATH"] = f"{_lp}:{os.environ.get('LD_LIBRARY_PATH', '')}"
ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "src"))
_RS = os.path.join(ROOT, "robosuite", "robosuite")
if os.path.exists(os.path.join(_RS, "__init__.py")) and _RS not in sys.path:
    sys.path.insert(0, _RS)

import numpy as np

ENV_NAME = "FactorySorting3_3FO3ERRPH7X9"
TRAJ = f"{ROOT}/recordings/{ENV_NAME}/trajectory_20260815_193350_OK.json"
OBJ = "green_tote_b01_upper"
LOW = "green_tote_b01_lower"

traj = json.load(open(TRAJ))
ev = next(e for e in traj["events"] if e["name"] == "grasp_start")
gframe = traj["frames"][ev["frame"]]

import robot_agent.skills.pick_up  # noqa: F401
from robot_agent.environments import RobosuiteBackend
from robot_agent.environments.robosuite_backend import (
    _set_base_world_yaw_direct, _set_base_xy_direct, _get_base_pose,
)
from robot_agent.task_subprocess_runner import SCENE_INPUT_OBJECT_MAP
from robosuite.environments.factory_sorting.transport_attachment import (
    sync_transport_attachment,
)

backend = RobosuiteBackend(env_name=ENV_NAME, camera="birdview",
                           drive_mode="direct", headless=True)
backend.reset()
dyn = {}
for obj_name, info in (getattr(backend.env, "material_metadata", {}) or {}).items():
    if isinstance(info, dict):
        pn = str(info.get("port_name") or "")
        if pn:
            dyn[pn] = obj_name
            if pn.startswith("input_"):
                dyn["line_" + pn.split("_", 1)[1]] = obj_name
            elif pn.startswith("line_"):
                dyn["input_" + pn.split("_", 1)[1]] = obj_name
dyn.update(SCENE_INPUT_OBJECT_MAP.get(ENV_NAME, {}))
backend.set_physics_grasp_config(device="auto", object_map=dyn)
backend.reset()

env = backend.env
robot = env.robots[0]
bq = gframe["base_pose"]["orientation_xyzw"]
_set_base_world_yaw_direct(env, robot, 2.0 * math.atan2(bq[2], bq[3]))
_set_base_xy_direct(env, robot, np.array(gframe["base_pose"]["position"][:2], dtype=float))
env.sim.forward()

print("=== 抓取(含抬升0.35+撤离) ===")
ok = backend.grasp_object_physics(ev["source"], object_name=OBJ)
print("grasp ->", ok)
base_now, yaw_now = backend.get_base_pose()
print("post-grasp base:", np.round(base_now, 3).tolist(), "yaw:", round(yaw_now, 3))

# 旧撤离路径上的关键点(取自旧轨迹 f285-370, yaw=180)
POINTS = [
    (12.90, 3.84), (12.90, 3.50), (12.90, 3.27), (12.90, 3.05),
    (12.82, 2.96), (12.75, 2.88), (12.66, 2.68), (12.65, 1.54),
]
LOWER = "green_tote_b01_lower"


def _dump_obj_geoms(name):
    """打印物体姿态 + 各碰撞 geom 的世界 z 范围(用于理解穿透)。"""
    q = env.sim.data.get_joint_qpos(f"{name}_joint0")
    print(f"   {name}: pos=({q[0]:.3f},{q[1]:.3f},{q[2]:.3f}) quat={np.round(q[3:7],3).tolist()}")
    for gid in range(env.sim.model.ngeom):
        nm = env.sim.model.geom_id2name(gid) or ""
        if nm.startswith(name + "_"):
            xpos = env.sim.data.geom_xpos[gid]
            xmat = env.sim.data.geom_xmat[gid].reshape(3, 3)
            size = env.sim.model.geom_size[gid]
            # AABB z 范围(近似: 中心 ± 旋转后各轴半长投影)
            dz = abs(xmat[:, 2]) @ size
            print(f"      {nm}: z [{xpos[2]-dz:.3f}, {xpos[2]+dz:.3f}]")


for pt in POINTS:
    _set_base_xy_direct(env, robot, np.array(pt))
    sync_transport_attachment(env)   # 持物随底盘
    env.sim.forward()
    q = env.sim.data.get_joint_qpos(f"{OBJ}_joint0")
    ql = env.sim.data.get_joint_qpos(f"{LOWER}_joint0")
    contacts = []
    for ci in range(env.sim.data.ncon):
        c = env.sim.data.contact[ci]
        if c.dist >= 0.005:
            continue
        g1 = env.sim.model.geom_id2name(c.geom1) or ""
        g2 = env.sim.model.geom_id2name(c.geom2) or ""
        if LOWER in g1 or LOWER in g2:
            pair = (g1, g2, round(float(c.dist) * 1000, 1))
            if pair not in contacts:
                contacts.append(pair)
    print(f"\n点 {pt}: 持物=({q[0]:.2f},{q[1]:.2f},{q[2]:.2f}) lower=({ql[0]:.2f},{ql[1]:.2f},{ql[2]:.2f})")
    if pt == (12.82, 2.96):
        _dump_obj_geoms(OBJ)
        _dump_obj_geoms(LOWER)
    seen = set()
    for g1, g2, d in contacts:
        key = tuple(sorted([g1, g2]))
        if key in seen:
            continue
        seen.add(key)
        print(f"   接触 {d:+.1f}mm  {g1} <-> {g2}")
    if not seen:
        print("   无接触")
backend.close()
