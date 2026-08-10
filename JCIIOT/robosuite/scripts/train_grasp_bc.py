"""Train a robomimic BC grasp policy from collected demonstrations (Task D).

Thin, opinionated wrapper around ``robomimic.scripts.train`` that bakes in the
factory-sorting grasp defaults via ``bc_grasp_config.json`` and exposes the few
knobs that matter for this competition (dataset, output dir, epochs). It mirrors
``robomimic/scripts/train.py::main`` exactly (config_factory → update → lock →
train) so behavior stays upstream-faithful, while adding ``--output-dir`` and
``--epochs`` overrides and §6.2 timestamped logging + exception handling.

Produces checkpoints under ``<output-dir>/<experiment.name>/models/``. Copy the
desired epoch checkpoint (e.g. ``model_epoch_150.pth``) to the path set in
``knowledge/robot_params.json`` → ``grasp_policy.checkpoint_path`` (default
``robosuite/robosuite/model_epoch_150.pth``) to deploy. The protected
``robosuite_backend.py`` reads that key (with ``checkpoint_fallback_path``
fallback).

Usage::

    python robosuite/scripts/train_grasp_bc.py \\
        --dataset robosuite/robosuite/models/assets/demonstrations_private/<ts>/grasp_l1_<ts>.hdf5 \\
        --config robosuite/scripts/bc_grasp_config.json \\
        --output-dir robosuite/runs --epochs 150

Prerequisites: a working MuJoCo install (``import mujoco`` succeeds) and the
collected HDF5 dataset. See ``robosuite/TASK_D_README.md``.

This script is a competition Task D artifact (custom-model training) and lives
under ``robosuite/`` per the CLAUDE.md allowance for training custom models.
"""

from __future__ import annotations

import argparse
import json
import sys
import traceback
from datetime import datetime
from pathlib import Path

# Make robomimic importable. robomimic is vendored at <project>/robomimic.
_HERE = Path(__file__).resolve().parent
_PROJECT_ROOT = _HERE.parents[1]            # .../JCIIOT
_ROBOMIMIC_ROOT = _PROJECT_ROOT / "robomimic"
for _p in (_PROJECT_ROOT, _ROBOMIMIC_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

DEFAULT_CONFIG = _HERE / "bc_grasp_config.json"


def _ts() -> str:
    return datetime.now().isoformat(timespec="seconds")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dataset", type=str, required=True,
                   help="path to collected grasp demonstrations (.hdf5)")
    p.add_argument("--config", type=str, default=str(DEFAULT_CONFIG),
                   help=f"path to BC config override json (default: {DEFAULT_CONFIG})")
    p.add_argument("--output-dir", type=str, default="robosuite/runs",
                   help="training output root (default: robosuite/runs)")
    p.add_argument("--name", type=str, default=None,
                   help="experiment name override (default: from config)")
    p.add_argument("--epochs", type=int, default=None,
                   help="override train.num_epochs (default: from config)")
    p.add_argument("--batch-size", type=int, default=None,
                   help="override train.batch_size")
    p.add_argument("--debug", action="store_true",
                   help="quick 2-epoch smoke run for debugging")
    p.add_argument("--resume", action="store_true",
                   help="resume training from latest checkpoint")
    return p.parse_args()


def main(args) -> int:
    print(f"[{_ts()}][train_grasp_bc] config={args.config} dataset={args.dataset} "
          f"output_dir={args.output_dir} epochs={args.epochs}")

    # ── Build config: upstream-faithful to robomimic/scripts/train.py::main ──
    from robomimic.config import config_factory
    import robomimic.utils.torch_utils as TorchUtils

    with open(args.config, "r") as f:
        ext_cfg = json.load(f)
    # Drop metadata keys (e.g. "_comment") that robomimic's config doesn't know.
    ext_cfg = {k: v for k, v in ext_cfg.items() if not k.startswith("_")}
    config = config_factory(ext_cfg["algo_name"])
    with config.values_unlocked():
        config.update(ext_cfg)

    # CLI overrides
    config.train.data = [{"path": args.dataset}]
    config.train.output_dir = args.output_dir
    if args.name is not None:
        config.experiment.name = args.name
    if args.epochs is not None:
        config.train.num_epochs = args.epochs
    if args.batch_size is not None:
        config.train.batch_size = args.batch_size

    device = TorchUtils.get_torch_device(try_to_use_cuda=config.train.cuda)

    if args.debug:
        config.unlock()
        config.lock_keys()
        config.experiment.epoch_every_n_steps = 3
        config.experiment.validation_epoch_every_n_steps = 3
        config.train.num_epochs = 2
        config.experiment.rollout.rate = 1
        config.experiment.rollout.n = 2
        config.experiment.rollout.horizon = 10
        config.train.output_dir = "/tmp/tmp_trained_models"

    config.lock()
    print(f"[{_ts()}][train_grasp_bc] algo={ext_cfg['algo_name']} "
          f"epochs={config.train.num_epochs} batch={config.train.batch_size} "
          f"device={device} experiment={config.experiment.name}")

    # ── Run training (import lazily so --help works without heavy deps) ──
    from robomimic.scripts.train import train

    res_str = "finished run successfully!"
    try:
        train(config, device=device, resume=args.resume)
    except Exception as e:  # noqa: BLE001 — surface full traceback, do not hide
        res_str = f"run failed with error:\n{e}\n\n{traceback.format_exc()}"
    print(f"[{_ts()}][train_grasp_bc] {res_str}")
    return 0 if res_str.startswith("finished") else 1


if __name__ == "__main__":
    sys.exit(main(parse_args()))
