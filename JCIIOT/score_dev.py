"""Dev scoring harness — run app.py's OFFICIAL scoring logic on a trajectory
file without launching Streamlit.

The scoring functions are extracted from ``app.py`` at runtime via AST, so
this harness always scores with the exact competition logic (no drift).

Usage::

    python score_dev.py <trajectory.json> [--task-index N] [--save]

Examples::

    python score_dev.py recordings/FactorySorting1_3FO3ERFHISEM/trajectory_x_OK.json --task-index 0
"""

from __future__ import annotations

import argparse
import ast
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

_APP_DIR = Path(__file__).resolve().parent
_SRC_DIR = _APP_DIR / "src"
for _p in (_SRC_DIR, _APP_DIR, _APP_DIR / "robosuite" / "robosuite", _APP_DIR / "robomimic"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

_TASK_CFG = json.loads((_APP_DIR / "knowledge" / "task_config.json").read_text(encoding="utf-8"))
_TASK_LIST = _TASK_CFG.get("tasks", [])

# Functions pulled verbatim from app.py (single source of truth).
_EXTRACT = [
    "_task_for_index",
    "_task_source_name",
    "_task_target_name",
    "_task_object_name",
    "_scene_prefix",
    "_scene_env_name",
    "_choose_map_files",
    "_json_safe",
    "_score_path_for_trajectory",
    "_write_score_file",
    "_event_success_value",
    "_trajectory_object_position",
    "_l5_match_object",
    "_l5_left_source_after_grasp",
    "_score_l5_multi_object",
    "_score_steps",
]


class _SessionState(dict):
    """Minimal stand-in for streamlit.session_state."""


class _StStub:
    def __init__(self, trajectory_path: str):
        self.session_state = _SessionState({"_last_trajectory": trajectory_path})


def _build_namespace(trajectory_path: str) -> dict:
    source = (_APP_DIR / "app.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    wanted = set(_EXTRACT)
    nodes = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in wanted
    ]
    found = {node.name for node in nodes}
    missing = wanted - found
    if missing:
        raise RuntimeError(f"app.py no longer defines: {sorted(missing)}")

    ns: dict[str, Any] = {
        "__name__": "score_dev",
        "np": np,
        "json": json,
        "time": time,
        "Path": Path,
        "Any": Any,
        "_APP_DIR": _APP_DIR,
        "_MAP_DIR": (
            _APP_DIR / "robosuite" / "robosuite" / "environments"
            / "factory_sorting" / "generated_maps"
        ),
        "_TASK_CFG": _TASK_CFG,
        "_TASK_LIST": _TASK_LIST,
        "L5_INPUT1_OBJECTS": (
            "white_tote_b01_left_center",
            "white_tote_b01_left_front",
            "white_tote_b01_left_back",
        ),
        "SCORE_RULE_VERSION": "grasp_success_gate_l5_multi_v2",
        "st": _StStub(trajectory_path),
    }
    module = ast.Module(body=nodes, type_ignores=[])
    exec(compile(module, str(_APP_DIR / "app.py"), "exec"), ns)
    return ns


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--task-index", type=int, default=None,
                        help="defaults to detection from the parent directory name")
    parser.add_argument("--save", action="store_true",
                        help="write the score_*.json next to the trajectory")
    args = parser.parse_args(argv)

    traj_path = args.trajectory.resolve()
    if not traj_path.exists():
        print(f"trajectory not found: {traj_path}", file=sys.stderr)
        return 2

    if args.task_index is None:
        env_to_index = {t["env_name"]: i for i, t in enumerate(_TASK_LIST)}
        task_index = env_to_index.get(traj_path.parent.name)
        if task_index is None:
            print("cannot infer task index; pass --task-index", file=sys.stderr)
            return 2
    else:
        task_index = args.task_index

    ns = _build_namespace(str(traj_path))
    t0 = time.perf_counter()
    details = ns["_score_steps"](task_index)
    elapsed = time.perf_counter() - t0
    details = ns["_json_safe"](details)

    print(json.dumps(details, indent=2, ensure_ascii=False))
    print(f"\nSCORE: {details.get('total', 0)} / {[10, 15, 20, 25, 30][task_index]}")

    if args.save:
        score_path = ns["_write_score_file"](
            task_index=task_index,
            details=details,
            trajectory_path=traj_path,
            status="OK",
            elapsed=elapsed,
        )
        print(f"saved: {score_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
