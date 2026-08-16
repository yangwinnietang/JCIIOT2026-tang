#!/usr/bin/env python3
"""Replay a saved trajectory JSON and render it to an MP4 video.

Uses the existing RobosuiteBackend.replay_trajectory() pipeline to
reconstruct robot + object state frame-by-frame in MuJoCo, then encodes
the rendered frames to H.264 MP4 via imageio-ffmpeg.

Usage:
    .venv/bin/python replay_to_video.py --trajectory <path> --camera birdview
    .venv/bin/python replay_to_video.py --trajectory <path> --camera all
    .venv/bin/python replay_to_video.py --level L1 --camera all

Environment:
    MUJOCO_GL=osmesa  (set automatically if not present)
    DISPLAY           (not required — headless offscreen rendering)
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

# ── env setup must happen before any mujoco/robosuite import ──
os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
_lib_path = "/etc/dsw/runtime/dynamic_libs/lib"
if Path(_lib_path).exists():
    os.environ.setdefault("LD_LIBRARY_PATH", f"{_lib_path}:{os.environ.get('LD_LIBRARY_PATH', '')}")

# ── repo root path setup (same pattern as test_scene_load.py) ──
ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
_SRC_DIR = ROOT / "src"
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

# ── robosuite namespace monkey-patch (same as app.py / test_scene_load.py) ──
_ROBOSUITE_INNER_DIR = ROOT / "robosuite" / "robosuite"
_ROBOSUITE_INNER = _ROBOSUITE_INNER_DIR / "__init__.py"
if _ROBOSUITE_INNER.exists():
    if str(_ROBOSUITE_INNER_DIR) not in sys.path:
        sys.path.insert(0, str(_ROBOSUITE_INNER_DIR))
    import robosuite as _rs
    _rs.__file__ = str(_ROBOSUITE_INNER)
    _rs.__path__ = [str(_ROBOSUITE_INNER_DIR)]
    with open(_ROBOSUITE_INNER, encoding="utf-8") as _f:
        _code = compile(_f.read(), str(_ROBOSUITE_INNER), "exec")
    exec(_code, _rs.__dict__)


# ── camera presets ──
CAMERAS = ["birdview", "robot0_robotview", "follow"]

# ── level → env_name mapping (from task_config.json) ──
LEVEL_MAP = {
    "L1": ("FactorySorting1_3FO3ERFHISEM", "FactorySorting1_3FO3ERFHISEM"),
    "L2": ("FactorySorting3_3FO3ERRPH7X9", "FactorySorting3_3FO3ERRPH7X9"),
    "L3": ("FactorySorting5_3FO3ERTPXEUT", "FactorySorting5_3FO3ERTPXEUT"),
    "L4": ("FactorySorting7_3FO3ERFKY9RN", "FactorySorting7_3FO3ERFKY9RN"),
    "L5": ("FactorySorting9_3FO3ERT2C5FP", "FactorySorting9_3FO3ERT2C5FP"),
}


def _find_latest_trajectory(env_name: str) -> Path | None:
    """Find the most recent OK trajectory for an env."""
    rec_dir = ROOT / "recordings" / env_name
    if not rec_dir.exists():
        return None
    files = sorted(
        rec_dir.glob("trajectory_*_OK.json"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    return files[0] if files else None


def _glitch_score(frame) -> float:
    """花屏检测分数：3x3 局部亮度标准差均值。正常渲染帧 0.7-6.3，损坏帧 24+。"""
    import numpy as np
    a = np.asarray(frame, dtype=np.float32)
    if a.ndim == 3:
        a = a.mean(axis=2)
    h, w = a.shape
    h3, w3 = h // 3 * 3, w // 3 * 3
    blocks = a[:h3, :w3].reshape(h3 // 3, 3, w3 // 3, 3)
    return float(blocks.std(axis=(1, 3)).mean())


GLITCH_THRESHOLD = 12.0


def _validate_frames(frames: list) -> list[int]:
    """返回损坏帧的下标列表（形状/ dtype 异常或花屏分数超标）。"""
    import numpy as np
    bad = []
    for i, f in enumerate(frames):
        a = np.asarray(f)
        if a.ndim != 3 or a.shape[2] != 3 or a.dtype != np.uint8:
            bad.append(i)
            continue
        if _glitch_score(a) > GLITCH_THRESHOLD:
            bad.append(i)
    return bad


def _render_frames_to_mp4(frames: list, output_path: Path, fps: int = 30) -> None:
    """Encode numpy RGB frames to H.264 MP4."""
    import imageio.v2 as imageio
    import numpy as np

    writer = imageio.get_writer(
        str(output_path),
        fps=fps,
        codec="libx264",
        quality=8,
        macro_block_size=1,
    )
    for frame in frames:
        writer.append_data(np.ascontiguousarray(frame, dtype=np.uint8))
    writer.close()


def _flatten_birdview(backend, z_scale: float = 2.0) -> None:
    """Raise the birdview camera and narrow its fovy to keep the SAME
    ground footprint while shrinking perspective parallax ∝ 1/z.

    An object held at 1.3m height near the scene edge was projected
    ~0.35m off its true XY (fovy=58° at z=32m), visually overlapping the
    robot/equipment in top-down videos.  Raising z by *z_scale* and
    setting fovy = 2·atan(tan(fovy/2)/z_scale) keeps the footprint
    pixel-identical but halves the shift; the view stays a faithful
    top-down map.
    """
    sim = backend.env.sim
    try:
        cam_id = sim.model.camera_name2id("birdview")
    except Exception:
        return
    fovy_old = float(sim.model.cam_fovy[cam_id])
    sim.model.cam_pos[cam_id][2] *= z_scale
    sim.model.cam_fovy[cam_id] = 2.0 * math.degrees(
        math.atan(math.tan(math.radians(fovy_old) / 2.0) / z_scale)
    )
    sim.forward()


def _render_trajectory_to_video(
    traj_path: Path,
    env_name: str,
    camera: str,
    output_path: Path,
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    frame_start: int | None = None,
    frame_end: int | None = None,
    frame_step: int = 1,
    birdview_flat: float = 0.0,
) -> int:
    """Replay trajectory and save as MP4. Returns frame count."""
    from robot_agent.environments import RobosuiteBackend

    # "follow" is not a real MuJoCo camera name — it's a mode inside
    # replay_trajectory that drives the free camera.  Use "birdview" for
    # env creation so _make_env succeeds, then pass "follow" to replay.
    init_camera = "birdview" if camera == "follow" else camera

    backend = RobosuiteBackend(
        env_name=env_name,
        camera=init_camera,
        drive_mode="direct",
        headless=True,
    )
    backend.reset()
    if birdview_flat and camera == "birdview":
        _flatten_birdview(backend, z_scale=birdview_flat)

    # Frame sub-sampling is done here (not in the harness): write a temp
    # trajectory with every Nth frame and replay that — replay sets
    # absolute qpos per frame so skipping frames is safe.
    actual_path = traj_path
    if frame_step and frame_step > 1:
        import json as _json
        import tempfile as _tempfile
        _data = _json.loads(traj_path.read_text(encoding="utf-8"))
        _data["frames"] = _data.get("frames", [])[:: int(frame_step)]
        _tmp = _tempfile.NamedTemporaryFile(
            mode="w", suffix="_stepped.json", delete=False, encoding="utf-8")
        _json.dump(_data, _tmp)
        _tmp.close()
        actual_path = Path(_tmp.name)

    # 分块流式渲染+编码：每块 ~120 帧（约 110MB）即渲即写，内存有界——
    # 整段累积帧列表（L5 单视角 ~3GB）曾在内存紧张时把 OSMesa 读回逼出花屏。
    # 每块渲染后逐帧校验（花屏 + 冻结）；损坏则**新建后端（全新 GL 上下文）**
    # 重渲该块——GL 上下文损坏是进程级的，原进程内重渲注定失败；仍坏则中止。
    import json as _json
    _frames_meta = _json.loads(Path(actual_path).read_text(encoding="utf-8")).get("frames", [])
    _total = len(_frames_meta)
    _s = int(frame_start) if frame_start is not None else 0
    _e = int(frame_end) if frame_end is not None else _total
    _e = min(_e, _total)
    if _e <= _s:
        print(f"  [WARN] empty frame range [{_s},{_e}) for camera={camera}")
        return 0

    def _expected_motion(cs, ce):
        """该块内底盘最大位移（米）——用于判定冻结帧是否异常。"""
        pts = [f.get("base_pose", {}).get("position", [0, 0]) for f in _frames_meta[cs:ce]]
        if len(pts) < 2:
            return 0.0
        import numpy as _np2
        a = _np2.array([p[:2] for p in pts], dtype=float)
        return float(_np2.abs(_np2.diff(a, axis=0)).max())

    def _frozen(frames):
        """连续帧几乎完全一致（GL 上下文损坏的典型表现：反复读回同一陈旧缓冲）。"""
        import numpy as _np2
        prev = None
        same = 0
        for fr in frames:
            a = _np2.asarray(fr, dtype=_np2.int16)
            if prev is not None and float(_np2.abs(a - prev).mean()) < 0.005:
                same += 1
            prev = a
        return same >= max(3, int(0.5 * len(frames)))

    import imageio.v2 as imageio
    writer = imageio.get_writer(
        str(output_path), fps=fps, codec="libx264", quality=8, macro_block_size=1,
    )
    written = 0
    chunk = 120
    try:
        for cs in range(_s, _e, chunk):
            ce = min(cs + chunk, _e)
            frames = None
            for attempt in (1, 2, 3):
                frames = backend.replay_trajectory(
                    str(actual_path),
                    output_gif=None,
                    camera=camera,
                    width=width,
                    height=height,
                    frame_start=cs,
                    frame_end=ce,
                )
                if not frames:
                    break
                bad = _validate_frames(frames)
                frozen = _frozen(frames) and _expected_motion(cs, ce) > 0.005
                if not bad and not frozen:
                    break
                why = "花屏" if bad else "冻结"
                print(f"  [WARN] 块[{cs},{ce}) {why}异常（第{attempt}次），"
                      + ("重建后端重渲该块..." if attempt < 3 else "三次均损坏！"), flush=True)
                if attempt < 3:
                    try:
                        backend.close()
                    except Exception:
                        pass
                    backend = RobosuiteBackend(
                        env_name=env_name, camera=init_camera,
                        drive_mode="direct", headless=True,
                    )
                    backend.reset()
                    if birdview_flat and camera == "birdview":
                        _flatten_birdview(backend, z_scale=birdview_flat)
                else:
                    backend.close()
                    writer.close()
                    output_path.unlink(missing_ok=True)
                    raise RuntimeError(
                        f"{camera} 块[{cs},{ce}) 三次渲染均损坏，已中止并删除 {output_path}")
            if not frames:
                break
            import numpy as _np
            for fr in frames:
                writer.append_data(_np.ascontiguousarray(fr, dtype=_np.uint8))
            written += len(frames)
            del frames
    finally:
        writer.close()
    backend.close()
    return written


def main():
    parser = argparse.ArgumentParser(
        description="Replay trajectory JSON to MP4 video",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Render latest L1 trajectory, all 3 cameras
  .venv/bin/python replay_to_video.py --level L1 --camera all

  # Render a specific trajectory file, birdview only
  .venv/bin/python replay_to_video.py \\
    --trajectory recordings/FactorySorting1_3FO3ERFHISEM/trajectory_20260814_230312_OK.json \\
    --camera birdview

  # Render a grasp segment (auto-detected)
  .venv/bin/python replay_to_video.py --level L1 --camera follow --grasp-only
        """,
    )
    parser.add_argument(
        "--trajectory", "-t",
        type=str,
        help="Path to trajectory JSON file (relative to repo root or absolute)",
    )
    parser.add_argument(
        "--level", "-l",
        type=str,
        choices=["L1", "L2", "L3", "L4", "L5"],
        help="Task level — auto-selects latest OK trajectory for that level",
    )
    parser.add_argument(
        "--camera", "-c",
        type=str,
        default="birdview",
        choices=["birdview", "robot0_robotview", "follow", "all"],
        help="Camera view (default: birdview). 'all' renders all 3",
    )
    parser.add_argument(
        "--grasp-only",
        action="store_true",
        help="Render only the grasp segment (auto-detected from events)",
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="Render the full trajectory even for robot0_robotview "
             "(which otherwise renders only the grasp segment by default)",
    )
    parser.add_argument(
        "--step", type=int, default=1,
        help="Render every Nth frame (default 1 = all frames). "
             "e.g. --step 3 renders 1/3 of the frames, ~3x faster",
    )
    parser.add_argument(
        "--width", type=int, default=640, help="Render width (default: 640)"
    )
    parser.add_argument(
        "--height", type=int, default=480, help="Render height (default: 480)"
    )
    parser.add_argument(
        "--fps", type=int, default=30, help="Output video FPS (default: 30)"
    )
    parser.add_argument(
        "--birdview-flat", type=float, default=2.0,
        help="Birdview de-parallax factor: raise camera z and narrow fovy by "
             "this factor (default 2.0; 0/1 = original camera).",
    )
    parser.add_argument(
        "--output-dir", "-o",
        type=str,
        default=None,
        help="Output directory (default: same dir as trajectory file)",
    )

    args = parser.parse_args()

    # ── resolve trajectory path ──
    if args.trajectory:
        traj_path = Path(args.trajectory)
        if not traj_path.is_absolute():
            traj_path = ROOT / traj_path
        if not traj_path.exists():
            print(f"ERROR: trajectory file not found: {traj_path}")
            sys.exit(1)
        # Infer env_name from parent dir
        env_name = traj_path.parent.name
    elif args.level:
        env_name = LEVEL_MAP[args.level][1]
        traj_path = _find_latest_trajectory(env_name)
        if traj_path is None:
            print(f"ERROR: no OK trajectory found for {args.level} in recordings/{env_name}/")
            sys.exit(1)
    else:
        print("ERROR: must specify --trajectory or --level")
        sys.exit(1)

    print(f"Trajectory: {traj_path.name}")
    print(f"Env:        {env_name}")

    # ── determine grasp range (reused per-camera) ──
    def _grasp_range(path: Path) -> tuple[int | None, int | None, str]:
        """Return (frame_start, frame_end, label) for the grasp segment."""
        import json
        data = json.loads(path.read_text(encoding="utf-8"))
        events = data.get("events", [])
        gs = None
        ge = None
        for ev in events:
            if ev.get("name") == "grasp_start":
                gs = int(ev["frame"])
            elif ev.get("name") == "grasp_end" and gs is not None:
                ge = int(ev["frame"]) + 1
                break
        total = len(data.get("frames", []))
        if gs is not None and ge is not None:
            return gs, ge, f" (grasp: frame {gs}-{ge} of {total})"
        return None, None, ""

    # ── determine cameras to render ──
    cameras = CAMERAS if args.camera == "all" else [args.camera]

    # ── output directory ──
    out_dir = Path(args.output_dir) if args.output_dir else traj_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── render each camera ──
    stem = traj_path.stem  # e.g. trajectory_20260814_230312_OK
    for cam in cameras:
        suffix = "" if cam == "birdview" else f"_{cam}"
        out_name = stem.replace("trajectory_", "replay_") + f"{suffix}.mp4"
        out_path = out_dir / out_name

        # Frame range: explicit --grasp-only, OR robot0_robotview defaults
        # to the grasp segment (full-render of first-person view is slow and
        # mostly static between operations).  --full overrides.
        frame_start = None
        frame_end = None
        grasp_label = ""
        use_grasp = args.grasp_only or (cam == "robot0_robotview" and not args.full)
        if use_grasp:
            gs, ge, grasp_label = _grasp_range(traj_path)
            if gs is not None:
                frame_start, frame_end = gs, ge
            else:
                print("  [WARN] no grasp_start/grasp_end events found, rendering full trajectory")

        print(f"\n[{cam}] → {out_path.name}{grasp_label}"
              + (f" (step={args.step})" if args.step > 1 else ""))
        t0 = time.perf_counter()
        n = _render_trajectory_to_video(
            traj_path=traj_path,
            env_name=env_name,
            camera=cam,
            output_path=out_path,
            width=args.width,
            height=args.height,
            fps=args.fps,
            frame_start=frame_start,
            frame_end=frame_end,
            frame_step=args.step,
            birdview_flat=args.birdview_flat,
        )
        elapsed = time.perf_counter() - t0
        size_kb = out_path.stat().st_size / 1024 if out_path.exists() else 0
        if n > 0:
            print(f"  ✓ {n} frames, {size_kb:.0f} KB, {elapsed:.1f}s")
        else:
            print(f"  ✗ no frames rendered")

    print("\nDone.")


if __name__ == "__main__":
    main()
