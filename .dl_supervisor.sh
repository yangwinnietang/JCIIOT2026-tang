#!/bin/bash
# Detached download supervisor: keeps retrying .dl_model.sh until the model
# is downloaded, verified, and deployed to robosuite/robosuite/model_epoch_150.pth.
# Survives Qwen Code session end (launched via setsid).
LOG=/mnt/workspace/JCIIOT2026/model_download_supervisor.log
TARGET=/mnt/workspace/JCIIOT2026/JCIIOT/robosuite/robosuite/model_epoch_150.pth
SHA_EXPECT=ef5910f6a9f6309b5ced617762dffeb1169a8b0cfcea892d158e6b483252169f

echo "[supervisor] started $(date)" >> "$LOG"
for round in $(seq 1 500); do
    # Already deployed and valid?
    if [ -f "$TARGET" ] && [ "$(stat -c%s "$TARGET" 2>/dev/null)" = "139543773" ]; then
        sha=$(sha256sum "$TARGET" | cut -d' ' -f1)
        if [ "$sha" = "$SHA_EXPECT" ]; then
            echo "[supervisor] model already deployed and verified, exiting $(date)" >> "$LOG"
            exit 0
        fi
    fi
    echo "[supervisor] round $round starting dl_model.sh $(date)" >> "$LOG"
    bash /mnt/workspace/JCIIOT2026/.dl_model.sh >> "$LOG" 2>&1
    if [ $? -eq 0 ]; then
        cp /mnt/workspace/JCIIOT2026/model_epoch_150.pth.download "$TARGET"
        echo "[supervisor] DOWNLOAD_VERIFIED_AND_DEPLOYED $(date)" >> "$LOG"
        exit 0
    fi
    echo "[supervisor] round $round failed; sleeping 120s $(date)" >> "$LOG"
    sleep 120
done
echo "[supervisor] gave up after 500 rounds $(date)" >> "$LOG"
