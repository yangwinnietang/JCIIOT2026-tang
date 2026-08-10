"""Merge multiple collected grasp-demonstration HDF5 files into one dataset.

robomimic's ``SequenceDataset`` enumerates ``f["data"].keys()`` and sorts them
as ``demo_<int>``, reading ``attrs["num_samples"]`` per demo. This script
concatenates the demo groups of several per-level files (e.g. ``grasp_l1_*.hdf5``,
``grasp_l3_*.hdf5``, …) into a single file with re-indexed ``demo_N`` keys so
one BC policy can be trained over all competition levels at once.

Usage::

    python robosuite/scripts/merge_grasp_datasets.py \
        --inputs a.hdf5 b.hdf5 c.hdf5 \
        --output merged_grasp_all.hdf5

This script is a competition Task D artifact and lives under ``robosuite/``.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import h5py


def _demo_sort_key(name: str) -> int:
    m = re.match(r"demo_(\d+)$", name)
    return int(m.group(1)) if m else 1 << 30


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--inputs", nargs="+", required=True, help="source hdf5 files")
    p.add_argument("--output", required=True, help="merged hdf5 output path")
    args = p.parse_args()

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    total = 0
    with h5py.File(out_path, "w") as out:
        out_data = out.create_group("data")
        wrote_attrs = False
        for src_path in args.inputs:
            with h5py.File(src_path, "r") as src:
                if not wrote_attrs:
                    for k, v in src["data"].attrs.items():
                        out_data.attrs[k] = v
                    wrote_attrs = True
                demo_keys = sorted(
                    (k for k in src["data"].keys() if re.match(r"demo_\d+$", k)),
                    key=_demo_sort_key,
                )
                for key in demo_keys:
                    new_key = f"demo_{total}"
                    src.copy(f"data/{key}", out_data, name=new_key)
                    # num_samples may be missing in some files; infer from actions shape
                    try:
                        ns = src[f"data/{key}"].attrs["num_samples"]
                    except KeyError:
                        ns = src[f"data/{key}/actions"].shape[0]
                    out_data[new_key].attrs["num_samples"] = ns
                    total += 1
                print(f"[merge] {src_path}: +{len(demo_keys)} demos (running total {total})")
        out_data.attrs["num_demos"] = total

    print(f"[merge] wrote {total} demos → {out_path}")


if __name__ == "__main__":
    main()
