#!/usr/bin/env python3
"""渲染轨迹指定帧为 PNG（用于证据复现与目检）。

相机逻辑与 robosuite_backend.replay_trajectory 完全一致：
  follow = 自由相机 distance=5.0, elevation=-35, azimuth=机器人 yaw, lookat=(base_xy, 1.0)
  其他   = 固定相机名（birdview / robot0_robotview ...）

用法:
  python render_frame.py <level> <trajectory.json> --frames 1140,1141 --camera follow [--birdview-flat 2.0] [--out /tmp/dir]
"""
import argparse
import math
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("level", choices=list(LEVEL_ENV))
    ap.add_argument("trajectory")
    ap.add_argument("--frames", required=True, help="逗号分隔帧号（轨迹帧号）")
    ap.add_argument("--camera", default="follow")
    ap.add_argument("--custom-cam", default=None,
                    help="自由相机: azimuth_offset_deg,distance,elevation_deg,lookat_z "
                         "（azimuth 相对机器人 yaw；例: 90,3,-15,1.0 = 侧面近距）")
    ap.add_argument("--highlight-geoms", default=None,
                    help="逗号分隔几何名（支持子串匹配），渲染时标红（取证可视化）")
    ap.add_argument("--birdview-flat", type=float, default=0.0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--out", default="/tmp/frames")
    args = ap.parse_args()

    import json
    import numpy as np
    import robot_agent.skills.pick_up  # noqa: F401  (应用运行时补丁)
    from robot_agent.environments import RobosuiteBackend

    data = json.loads(Path(args.trajectory).read_text(encoding="utf-8"))
    frames = data["frames"]
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    backend = RobosuiteBackend(env_name=LEVEL_ENV[args.level],
                               camera="birdview" if args.camera == "follow" else args.camera,
                               drive_mode="direct", headless=True)
    backend.reset()
    sim = backend.env.sim

    if args.highlight_geoms:
        toks = [t for t in args.highlight_geoms.split(",") if t]
        n_hl = 0
        for gid in range(sim.model.ngeom):
            gn = sim.model.geom_id2name(gid) or ""
            if any(t in gn for t in toks):
                sim.model.geom_rgba[gid] = [1.0, 0.0, 0.0, 1.0]
                n_hl += 1
        sim.forward()
        print(f"highlighted {n_hl} geoms red")

    if args.birdview_flat and args.camera == "birdview":
        cam_id = sim.model.camera_name2id("birdview")
        fovy_old = float(sim.model.cam_fovy[cam_id])
        sim.model.cam_pos[cam_id][2] *= args.birdview_flat
        sim.model.cam_fovy[cam_id] = 2.0 * math.degrees(
            math.atan(math.tan(math.radians(fovy_old) / 2.0) / args.birdview_flat))
        sim.forward()

    # 复用 replay_trajectory 的状态恢复逻辑：直接调用 replay 并只取目标帧
    # （replay 是逐帧绝对置位，渲染区间即为所求）
    idxs = [int(x) for x in args.frames.split(",")]
    lo, hi = min(idxs), max(idxs) + 1

    if args.custom_cam:
        # 自定义自由相机：手动置位 + 自由相机渲染
        az_off, dist, elev, lookz = [float(x) for x in args.custom_cam.split(",")]
        import mujoco
        from robosuite.environments.factory_sorting.load_factory_sorting_evalization import (
            get_base_world_pose,
        )
        from robot_agent.environments.robosuite_backend import (
            _set_base_xy_direct, _set_base_world_yaw_direct,
        )
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
        for fi in idxs:
            frame = frames[fi]
            bp = frame["base_pose"]["position"]
            bq = frame["base_pose"]["orientation_xyzw"]
            yaw = float(np.arctan2(2 * bq[3] * bq[2], 1 - 2 * bq[2] * bq[2]))
            # 与 replay_trajectory 同序：先 yaw 后 xy
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
            rctx = sim._render_context_offscreen
            rctx.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            rctx.cam.lookat[:] = [float(bp[0]), float(bp[1]), lookz]
            rctx.cam.distance = dist
            rctx.cam.azimuth = float(math.degrees(yaw)) + az_off
            rctx.cam.elevation = elev
            img = sim.render(camera_name=None, width=args.width, height=args.height, depth=False)
            img = np.array(img[::-1], dtype=np.uint8)
            from PIL import Image
            p = out_dir / f"{args.level}_custom{int(az_off)}_f{fi}.png"
            Image.fromarray(img).save(p)
            print(f"saved {p}")
        backend.close()
        return 0

    imgs = backend.replay_trajectory(
        args.trajectory, output_gif=None, camera=args.camera,
        width=args.width, height=args.height,
        frame_start=lo, frame_end=hi,
    )
    for i, img in enumerate(imgs):
        fi = lo + i
        if fi not in idxs:
            continue
        from PIL import Image
        p = out_dir / f"{args.level}_{args.camera}_f{fi}.png"
        Image.fromarray(np.asarray(img)).save(p)
        print(f"saved {p}")
    backend.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
