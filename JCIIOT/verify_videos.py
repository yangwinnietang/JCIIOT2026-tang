#!/usr/bin/env python3
"""视频完整性验证 — 检测花屏/黑屏/冻结帧。

原理：花屏帧（编码/渲染损坏）的 3x3 局部亮度标准差均值远高于正常渲染帧。
实测标定：正常帧 0.7-6.3，花屏帧 24.7-28.5 → 阈值 12。

用法:
  python verify_videos.py <video.mp4> [更多视频...]
  python verify_videos.py --dir ../videos
退出码: 0 = 全部通过, 1 = 有异常
"""
import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image

GLITCH_THRESHOLD = 12.0


def glitch_score(arr: np.ndarray) -> float:
    a = arr.astype(np.float32)
    if a.ndim == 3:
        a = a.mean(axis=2)
    h, w = a.shape
    h3, w3 = h // 3 * 3, w // 3 * 3
    blocks = a[:h3, :w3].reshape(h3 // 3, 3, w3 // 3, 3)
    return float(blocks.std(axis=(1, 3)).mean())


def probe(path: Path) -> dict:
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-print_format", "json",
         "-show_entries", "format=duration:stream=nb_frames,width,height,codec_name",
         str(path)], capture_output=True, text=True)
    info = json.loads(out.stdout or "{}")
    st = (info.get("streams") or [{}])[0]
    return {
        "duration": float(info.get("format", {}).get("duration", 0)),
        "nb_frames": int(st.get("nb_frames", 0)),
        "width": st.get("width", 0), "height": st.get("height", 0),
        "codec": st.get("codec_name", "?"),
    }


def sample_frames(path: Path, every_s: float = 2.0):
    """用 ffmpeg 每隔 every_s 秒抽一帧，生成 (t, ndarray)。"""
    meta = probe(path)
    dur = meta["duration"]
    ts = np.arange(0.0, max(dur - 0.05, 0.05), every_s)
    for t in ts:
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tf:
            tmp = tf.name
        r = subprocess.run(
            ["ffmpeg", "-v", "error", "-ss", f"{t:.2f}", "-i", str(path),
             "-frames:v", "1", "-y", tmp],
            capture_output=True, text=True, timeout=60)
        if r.returncode != 0 or not Path(tmp).exists() or Path(tmp).stat().st_size == 0:
            yield float(t), None
            continue
        try:
            yield float(t), np.asarray(Image.open(tmp))
        finally:
            Path(tmp).unlink(missing_ok=True)


def check_video(path: Path) -> bool:
    meta = probe(path)
    worst, worst_t, n_bad, n = 0.0, 0.0, 0, 0
    black = 0
    prev = None
    frozen = 0
    for t, frame in sample_frames(path):
        n += 1
        if frame is None:
            print(f"  ✗ t={t:.1f}s 解码失败")
            n_bad += 1
            continue
        s = glitch_score(frame)
        if float(frame.mean()) < 2.0:
            black += 1
        if prev is not None:
            import numpy as _np
            a = _np.asarray(frame, dtype=_np.int16)
            b = _np.asarray(prev, dtype=_np.int16)
            if a.shape == b.shape:
                diff = float(_np.abs(a - b).mean())
                if diff < 0.05:
                    frozen += 1
                    if frozen <= 3:
                        print(f"  ✗ t={t:.1f}s 冻结帧（与上一抽样几乎一致, diff={diff:.4f}）")
        prev = frame
        if s > worst:
            worst, worst_t = s, t
        if s > GLITCH_THRESHOLD:
            print(f"  ✗ t={t:.1f}s 花屏分数 {s:.1f} > {GLITCH_THRESHOLD}")
            n_bad += 1
    # 允许最多 1 次冻结抽样（长视频末段静置属正常）；≥2 次即整段冻结
    ok = n_bad == 0 and black == 0 and frozen <= 1
    status = "✓" if ok else "✗"
    print(f"{status} {path.name}: {meta['codec']} {meta['width']}x{meta['height']} "
          f"{meta['nb_frames']}帧 {meta['duration']:.1f}s | 抽样{n}帧 花屏max={worst:.1f}@t={worst_t:.1f}s"
          + (f" 黑屏帧{black}" if black else "")
          + (f" 冻结抽样{frozen}" if frozen else ""))
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("videos", nargs="*")
    ap.add_argument("--dir", default=None, help="检查目录下全部 mp4")
    args = ap.parse_args()
    paths = []
    if args.dir:
        paths = sorted(Path(args.dir).glob("*.mp4"))
    paths += [Path(v) for v in args.videos]
    if not paths:
        print("无视频可检查")
        return 2
    all_ok = True
    for p in paths:
        try:
            all_ok &= check_video(p)
        except Exception as e:
            print(f"✗ {p.name}: 检查异常 {e}")
            all_ok = False
    print("\n总结:", "全部通过" if all_ok else "存在异常视频")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
