"""Scene-parameterized grasp-demonstration collector for FactorySorting (Task D).

Generalizes ``load_factory_sorting_1_3fo3erfhisem_collect.py`` (which is hardcoded
to scene 1) to any of the five competition levels (1/3/5/7/9). It reuses the
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
    # (--level 1 3 5 7 9), so one invocation can collect every level.
    p.add_argument("--level", type=int, required=True, nargs="+",
                   choices=[1, 3, 5, 7, 9],
                   help="competition level(s) to collect: 1/3/5/7/9 (one or more)")
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
    return p.parse_args()


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
