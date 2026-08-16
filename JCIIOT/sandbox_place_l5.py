#!/usr/bin/env python3
"""L5 放置沙盒 — 复现并验证修复"放置摆动撞已放箱"(基线: −36mm 穿透, 推挤 0.64m, 末态重叠 0.295m)。

场景搭建(取自基线满分轨迹的真实位姿):
  tote2: 1 箱已放, 放第 2 箱;   tote3: 2 箱已放, 放第 3 箱
判定: 已放箱位移 <0.05m; 新落点与已放箱间距 >0.45m; place 成功; 无 judge 碰撞。

用法:  .venv/bin/python sandbox_place_l5.py [tote2|tote3]   (默认 tote2)
"""
import json
import math
import os
import sys
import time

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

ENV_NAME = "FactorySorting9_3FO3ERT2C5FP"
TRAJ = f"{ROOT}/recordings/{ENV_NAME}/trajectory_20260816_012725_OK.json"
TARGET = "aux_output_1"

SCENARIOS = {
    "tote2": {
        "placed": {"white_tote_b01_left_center": 3200},
        "held": "white_tote_b01_left_front",
        "ref_frame": 4100,
    },
    "tote3": {
        "placed": {"white_tote_b01_left_center": 4900, "white_tote_b01_left_front": 4900},
        "held": "white_tote_b01_left_back",
        "ref_frame": 5800,
    },
}


def main(scn: str) -> bool:
    cfg = SCENARIOS[scn]
    held = cfg["held"]
    traj = json.load(open(TRAJ))
    frames = traj["frames"]
    ref = frames[cfg["ref_frame"]]
    placed_poses = {name: frames[fidx]["object_positions"][name][:7]
                    for name, fidx in cfg["placed"].items()}
    held_pose = ref["object_positions"][held][:7]
    print(f"场景 {scn}: 参考帧 f{cfg['ref_frame']} base={ref['base_pose']['position'][:2]}")
    for n, p in placed_poses.items():
        print(f"  已放 {n}: {np.round(p[:3],3).tolist()}")
    print(f"  持 {held}: {np.round(held_pose[:3],3).tolist()}")

    import robot_agent.skills.pick_up  # noqa: F401  (应用补丁)
    from robot_agent.core.map_loader import load_map_files
    from robot_agent.core.scene_context import SceneContext
    from robot_agent.core.types import ExecutionContext
    from robot_agent.environments import RobosuiteBackend
    from robot_agent.environments.robosuite_backend import (
        _set_base_world_yaw_direct, _set_base_xy_direct,
    )
    from robot_agent.task_subprocess_runner import (
        SCENE_INPUT_OBJECT_MAP, _choose_map_files,
    )
    from robosuite.environments.factory_sorting.transport_attachment import (
        capture_transport_attachment,
    )

    backend = RobosuiteBackend(env_name=ENV_NAME, camera="birdview",
                               drive_mode="direct", headless=True)
    backend.reset()
    dynamic_input_object_map: dict[str, str] = {}
    raw_metadata = getattr(backend.env, "material_metadata", {}) or {}
    for obj_name, info in raw_metadata.items():
        if isinstance(info, dict):
            port_name = str(info.get("port_name") or "")
            if port_name:
                dynamic_input_object_map[port_name] = obj_name
                if port_name.startswith("input_"):
                    dynamic_input_object_map["line_" + port_name.split("_", 1)[1]] = obj_name
                elif port_name.startswith("line_"):
                    dynamic_input_object_map["input_" + port_name.split("_", 1)[1]] = obj_name
    full_object_map = dict(dynamic_input_object_map)
    full_object_map.update(SCENE_INPUT_OBJECT_MAP.get(ENV_NAME, {}))
    backend.set_physics_grasp_config(device="auto", object_map=full_object_map)
    backend.reset()

    import pathlib
    semantic, grid_file = _choose_map_files(pathlib.Path(ROOT), 4)
    scene, grid = load_map_files(semantic, grid_file)
    scene_ctx = SceneContext.from_semantic_map(scene)
    backend._scene_context = scene_ctx

    env = backend.env
    robot = env.robots[0]
    bq = ref["base_pose"]["orientation_xyzw"]
    yaw = 2.0 * math.atan2(bq[2], bq[3])
    _set_base_world_yaw_direct(env, robot, yaw)
    _set_base_xy_direct(env, robot, np.array(ref["base_pose"]["position"][:2], dtype=float))

    def _set_obj(name, pose7):
        for sfx in ("_joint0", "_free"):
            try:
                env.sim.data.set_joint_qpos(f"{name}{sfx}", list(pose7))
                env.sim.data.set_joint_qvel(f"{name}{sfx}", np.zeros(6))
                return
            except Exception:
                continue
        raise RuntimeError(f"no joint for {name}")
    for n, p in placed_poses.items():
        _set_obj(n, p)
    _set_obj(held, held_pose)
    env.sim.forward()
    att = capture_transport_attachment(env, held)
    backend._held_crate_name = held
    print(f"attachment: rel={np.round(att['relative_xy'],3).tolist()} z={att['world_z']:.3f}")

    placed_before = {n: np.array(p[:3]) for n, p in placed_poses.items()}

    from robot_agent.skills.place_down import PlaceDownSkill
    skill = PlaceDownSkill(backend=backend, scene_context=scene_ctx)
    ctx = ExecutionContext(task=f"place the tote at {TARGET}",
                           metadata={"inputs": {"target": TARGET}})
    t0 = time.time()
    result = skill.run(ctx)
    print(f"\nplace_down -> {result.success} ({time.time()-t0:.0f}s)\n  {result.message}")

    def _obj_xyz(name):
        for sfx in ("_joint0", "_free"):
            try:
                q = env.sim.data.get_joint_qpos(f"{name}{sfx}")
                return np.array(q[:3])
            except Exception:
                continue
        return None

    ok = bool(result.success)
    for n, before in placed_before.items():
        d = float(np.linalg.norm(_obj_xyz(n) - before))
        good = d < 0.05
        ok = ok and good
        print(f"已放 {n} 位移: {d:.3f}m  ({'OK' if good else '✗ FAIL'} <0.05)")
    hp = _obj_xyz(held)
    for n in placed_before:
        gap = float(np.linalg.norm(hp[:2] - _obj_xyz(n)[:2]))
        good = gap > 0.45
        ok = ok and good
        print(f"新落点 vs {n.split('_')[-1]}: {gap:.3f}m  ({'OK' if good else '✗ FAIL'} >0.45)")
    judge = bool(getattr(env, "has_judge_collision", False))
    ok = ok and not judge
    print(f"judge 碰撞: {judge}")
    print(f"\n{'✓✓ L5 放置沙盒通过' if ok else '✗✗ 未通过'} ({scn})")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1] if len(sys.argv) > 1 else "tote2") else 1)
