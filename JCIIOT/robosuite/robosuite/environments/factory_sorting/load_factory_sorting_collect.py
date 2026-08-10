"""Scene-parameterized grasp-demonstration collector for FactorySorting (Task D).

Generalizes ``load_factory_sorting_1_3fo3erfhisem_collect.py`` (which is hardcoded
to scene 1) to any of the five competition levels (L1..L5 → scenes 1/3/5/7/9). It reuses the
scene-1 collector's entire OSC two-arm grasp pipeline verbatim — only the
env name, target object, and robot base pose are derived per level from
``knowledge/task_config.json`` (the canonical source/target/object map and
``grasp_poses``).

Per-level derivation:
  - env_name     = task_config["tasks"][level]["env_name"]   (e.g. FactorySorting1_3FO3ERFHISEM)
  - object_name  = task_config["tasks"][level]["object"]     (the level's source object)
  - robot_base   = grasp_poses[source]["pos"]  +  [0, 0, grasp_poses[source]["yaw"]]

For L1 this reproduces the scene-1 collector exactly (object
``line_5_container_h01_near``, base ``[8.0, 4.6, 0.0]``, yaw ~π), which
validates the parameterization.

Usage (run headless with a working MUJOCO_GL backend, e.g. osmesa)::

    MUJOCO_GL=osmesa python -m robosuite.environments.factory_sorting.load_factory_sorting_collect \
        --level 1 --num-rollouts 30 --no-render --output-name grasp_l1

This script is a competition Task D artifact (custom-model training) and lives
under ``robosuite/`` per the CLAUDE.md allowance for training custom models.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]            # .../JCIIOT/robosuite (for `import robosuite`)
PROJECT_ROOT = Path(__file__).resolve().parents[4]   # .../JCIIOT
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# Reuse the proven scene-1 collector as a library (its grasp pipeline, HDF5
# writer, env kwargs, etc. are all reusable).
from robosuite.environments.factory_sorting import (  # noqa: E402
    load_factory_sorting_1_3fo3erfhisem_collect as coll,
)

TASK_CONFIG_PATH = PROJECT_ROOT / "knowledge" / "task_config.json"


def _level_config(level: int) -> dict:
    """Return {env_name, object, source, base_pos, base_ori} for a competition level."""
    tc = json.loads(TASK_CONFIG_PATH.read_text(encoding="utf-8"))
    task = next((t for t in tc.get("tasks", []) if t.get("level") == f"L{level}"), None)
    if task is None:
        raise ValueError(f"No task_config entry for level L{level}")
    source = task["source"]
    obj = task["object"]
    env_name = task["env_name"]
    gp = (tc.get("grasp_poses") or {}).get(source) or {}
    gp_pos = gp.get("pos") or [0.0, 0.0, 0.0]
    gp_yaw = float(gp.get("yaw") if gp.get("yaw") is not None else 0.0)
    base_pos = [float(gp_pos[0]), float(gp_pos[1]), float(gp_pos[2] if len(gp_pos) > 2 else 0.0)]
    base_ori = [0.0, 0.0, gp_yaw]
    return {
        "env_name": env_name,
        "object": obj,
        "source": source,
        "base_pos": base_pos,
        "base_ori": base_ori,
    }


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    # nargs="+" accepts both a single level (--level 1) and a batch
    # (--level 1 2 3 4 5), so one invocation can collect every level.
    # NOTE: these are competition LEVEL numbers (task_config "L{level}"),
    # not scene numbers — L1..L5 map to scenes 1/3/5/7/9 respectively.
    p.add_argument("--level", type=int, required=True, nargs="+",
                   choices=[1, 2, 3, 4, 5],
                   help="competition level(s) to collect: 1..5 (one or more)")
    p.add_argument("--num-rollouts", type=int, default=30)
    p.add_argument("--output-name", type=str, default=None,
                   help="default: grasp_l<level> (per level)")
    p.add_argument("--no-render", action="store_true", default=True,
                   help="headless (default true for sandbox collection). "
                        "Offscreen camera obs still need MUJOCO_GL=osmesa|egl.")
    p.add_argument("--render", dest="no_render", action="store_false",
                   help="enable on-screen rendering (needs a display)")
    p.add_argument("--object-name", type=str, default=None,
                   help="override target object (default: from task_config)")
    p.add_argument("--robot-base-pos", type=float, nargs=3, default=None)
    p.add_argument("--robot-base-ori", type=float, nargs=3, default=None)
    p.add_argument("--directory", type=str, default=None)
    p.add_argument("--seed", type=int, default=None)
    # Forwarded grasp-geometry knobs (needed for tall totes whose rim sits
    # above the default safe-z, so the XY traverse clears the object).
    p.add_argument("--safe-z", type=float, default=None)
    p.add_argument("--site-above-clearance", type=float, default=None)
    p.add_argument("--site-below-offset", type=float, default=None)
    p.add_argument("--arrival-tolerance", type=float, default=None)
    p.add_argument("--gripper-end-arrival-tolerance", type=float, default=None)
    p.add_argument("--wall-clamp", type=str, default=None,
                   help="'span,max_reach': clamp grasp targets into the reachable "
                        "part of the object wall (totes whose deep grasp sites are "
                        "outside the arm workspace from any collision-free base pose)")
    p.add_argument("--xwall-grasp", type=str, default=None,
                   help="'inset,span_half': re-target the grasp onto the object's "
                        "+x (aisle-facing) wall instead of the nominal -y wall. "
                        "Used for totes whose nominal sites are unreachable; the +x "
                        "wall reproduces the proven L1-style symmetric two-arm pinch.")
    return p.parse_args()


def _install_xwall_grasp(inset: float, span_half: float) -> None:
    """Patch get_target_positions to place targets on the object's +x wall.

    Some totes sit deep inside the collision-proxy region with their nominal
    grasp sites on the -y wall, unreachable by the far arm from any
    collision-free base pose. Their +x wall faces the aisle, though, and a
    symmetric two-arm pinch there reproduces the proven container geometry.
    Targets are set to (obj_x + inset, obj_y ± span_half) at the nominal site
    height.
    """
    import numpy as _np
    _orig = coll.get_target_positions

    def patched(env, object_name, site_below_offset):
        targets, names = _orig(env, object_name, site_below_offset)
        try:
            md = getattr(env, "material_metadata", {}).get(object_name, {})
            jn = md.get("joint_name") or f"{object_name}_free"
            qpos = _np.asarray(env.sim.data.get_joint_qpos(jn), dtype=float)
            obj_x, obj_y = float(qpos[0]), float(qpos[1])
            z_ref = float(next(iter(targets.values()))[2])
            new = {}
            for k, p in targets.items():
                p = _np.array(p, dtype=float)
                p[0] = obj_x + inset
                p[1] = obj_y + (span_half if k.lower().startswith("right") else -span_half)
                p[2] = z_ref
                new[k] = p
            print(f"[xwall_grasp] targets on +x wall: obj=({obj_x:.3f},{obj_y:.3f}) "
                  f"x={obj_x + inset:.3f} y_span=±{span_half}", flush=True)
            return new, names
        except Exception as exc:
            print(f"[xwall_grasp] patch error (keeping nominal targets): {exc}", flush=True)
            return targets, names

    coll.get_target_positions = patched


def _install_wall_clamp(span: float, max_reach: float) -> None:
    """Patch the scene-1 collector's get_target_positions.

    Some totes sit deep inside the scene's collision-proxy region: their
    nominal grasp sites are farther from every collision-free base pose than
    the arms can reach. The wall they sit on extends toward the aisle, though,
    so this patch shifts the grasp targets along that wall into the reachable
    window (keeping the two grippers ``span`` apart, biased aisle-side). The
    demonstrations then teach a wall-pinch grasp at the reachable positions.
    """
    import numpy as _np
    _orig = coll.get_target_positions

    def patched(env, object_name, site_below_offset):
        targets, names = _orig(env, object_name, site_below_offset)
        try:
            from robosuite.environments.factory_sorting.load_factory_sorting_evalization import (
                get_base_world_pose,
            )
            base_xy, _yaw = get_base_world_pose(env)
            pts = {k: _np.asarray(v, dtype=float) for k, v in targets.items()}
            if all(_np.linalg.norm(p[:2] - _np.asarray(base_xy)) <= max_reach
                   for p in pts.values()):
                return targets, names  # all reachable — keep nominal targets
            ys = [float(p[1]) for p in pts.values()]
            if max(ys) - min(ys) < 0.05:  # wall parallel to x at constant y
                wall_y = sum(ys) / len(ys)
                dy = wall_y - float(base_xy[1])
                w = (max(max_reach, 1e-3) ** 2 - dy * dy) ** 0.5 if abs(dy) < max_reach else 0.1
                lo = float(base_xy[0]) - w
                hi = float(base_xy[0]) + w
                xs = sorted(float(p[0]) for p in pts.values())
                right_x = min(xs[-1], hi - 0.02)
                left_x = right_x - span
                if left_x < lo + 0.02:
                    left_x = lo + 0.02
                    right_x = left_x + span
                new = {}
                for k, p in targets.items():
                    p = _np.array(p, dtype=float)
                    p[0] = left_x if k.lower().startswith("left") else right_x
                    p[1] = wall_y
                    new[k] = p
                print(f"[wall_clamp] targets clamped: left_x={left_x:.3f} "
                      f"right_x={right_x:.3f} wall_y={wall_y:.3f} "
                      f"base=({_np.round(base_xy, 3).tolist()})", flush=True)
                return new, names
        except Exception as exc:
            print(f"[wall_clamp] patch error (keeping nominal targets): {exc}", flush=True)
        return targets, names

    coll.get_target_positions = patched


def _collect_one(level: int, args) -> None:
    """Drive the scene-1 collector for a single level (timestamped)."""
    from datetime import datetime
    cfg = _level_config(level)
    env_name = cfg["env_name"]
    obj = args.object_name or cfg["object"]
    base_pos = args.robot_base_pos or cfg["base_pos"]
    base_ori = args.robot_base_ori or cfg["base_ori"]
    output_name = args.output_name or f"grasp_l{level}"
    ts = datetime.now().isoformat(timespec="seconds")
    print(f"[{ts}][collect] level=L{level} env={env_name} object={obj} "
          f"base_pos={base_pos} base_ori={base_ori} rollouts={args.num_rollouts}")

    # Patch the scene-1 collector's hardcoded env name, then drive its main()
    # with level-derived args via sys.argv. All 5 FactorySorting env classes
    # are registered via factory_sorting/__init__.py, so suite.make() resolves
    # any level's env_name.
    coll.DEFAULT_ENV_NAME = env_name

    if args.wall_clamp:
        _span, _reach = (float(v) for v in args.wall_clamp.split(","))
        _install_wall_clamp(_span, _reach)
    if args.xwall_grasp:
        _inset, _half = (float(v) for v in args.xwall_grasp.split(","))
        _install_xwall_grasp(_inset, _half)

    coll_argv = [
        "collect",
        "--num-rollouts", str(args.num_rollouts),
        "--object-name", str(obj),
        "--robot-base-pos", *[f"{v}" for v in base_pos],
        "--robot-base-ori", *[f"{v}" for v in base_ori],
        "--output-name", output_name,
        "--no-show-object-sites",
    ]
    if args.directory:
        coll_argv += ["--directory", args.directory]
    if args.seed is not None:
        coll_argv += ["--seed", str(args.seed)]
    if args.safe_z is not None:
        coll_argv += ["--safe-z", str(args.safe_z)]
    if args.site_above_clearance is not None:
        coll_argv += ["--site-above-clearance", str(args.site_above_clearance)]
    if args.site_below_offset is not None:
        coll_argv += ["--site-below-offset", str(args.site_below_offset)]
    if args.arrival_tolerance is not None:
        coll_argv += ["--arrival-tolerance", str(args.arrival_tolerance)]
    if args.gripper_end_arrival_tolerance is not None:
        coll_argv += ["--gripper-end-arrival-tolerance", str(args.gripper_end_arrival_tolerance)]
    if args.no_render:
        coll_argv.append("--no-render")

    sys.argv = coll_argv
    coll.main()


def main():
    args = parse_args()
    for level in args.level:
        _collect_one(level, args)


if __name__ == "__main__":
    main()
