#!/bin/bash
# Parallel chunked downloader for model_epoch_150.pth (GitHub LFS) with mirror rotation.
# github.com direct is currently blocked from this host; public GitHub proxies are used.
# Integrity: each attempt must return HTTP 206 (exact range) before appending;
# final file verified by sha256 before deployment (see .dl_supervisor.sh).
REL_PATH="JCIIOT2026/JCIIOT2026/raw/refs/heads/master/JCIIOT/robosuite/robosuite/model_epoch_150.pth"
MIRROR_LIST="https://ghfast.top/https://github.com https://ghproxy.net/https://github.com https://gh-proxy.com/https://github.com"
TOTAL=139543773
N=16
DIR=/mnt/workspace/JCIIOT2026/.model_chunks
OUT=/mnt/workspace/JCIIOT2026/model_epoch_150.pth.download
SHA_EXPECT=ef5910f6a9f6309b5ced617762dffeb1169a8b0cfcea892d158e6b483252169f
export REL_PATH MIRROR_LIST TOTAL DIR
CHUNK=$(( (TOTAL + N - 1) / N ))
export CHUNK
mkdir -p "$DIR"

seq 0 $((N-1)) | xargs -P 6 -I{} bash -c '
  i={}
  read -ra MIRRORS <<< "$MIRROR_LIST"
  start=$(( i * CHUNK ))
  end=$(( start + CHUNK - 1 ))
  [ "$end" -ge "$TOTAL" ] && end=$(( TOTAL - 1 ))
  want=$(( end - start + 1 ))
  out="$DIR/chunk_$(printf "%02d" "$i")"
  part="$out.part"
  if [ -f "$out" ] && [ "$(stat -c%s "$out")" -eq "$want" ]; then
    echo "chunk $i already complete"
    exit 0
  fi
  touch "$part"
  m=$i
  for try in $(seq 1 1200); do
    s=$(stat -c%s "$part" 2>/dev/null || echo 0)
    if [ "$s" -ge "$want" ]; then break; fi
    prefix="${MIRRORS[$(( m % ${#MIRRORS[@]} ))]}"
    m=$(( m + 1 ))
    tmp="$part.tmp"
    code=$(curl -sSL --connect-timeout 8 --max-time 45 \
      -w "%{http_code}" -o "$tmp" \
      --range $(( start + s ))-$end "$prefix/$REL_PATH" 2>/dev/null)
    if [ "$code" = "206" ] && [ -s "$tmp" ]; then
      cat "$tmp" >> "$part"
    fi
    rm -f "$tmp"
    sleep 0.2
  done
  s=$(stat -c%s "$part" 2>/dev/null || echo 0)
  if [ "$s" -eq "$want" ]; then
    mv "$part" "$out"
    echo "chunk $i done"
    exit 0
  fi
  echo "chunk $i FAILED at $s/$want"
  exit 1
'
status=$?
if [ "$status" -ne 0 ]; then
  echo "DOWNLOAD_CHUNKS_FAILED"
  exit 1
fi

cat "$DIR"/chunk_* > "$OUT"
size=$(stat -c%s "$OUT")
sha=$(sha256sum "$OUT" | cut -d" " -f1)
echo "assembled size=$size/$TOTAL sha256=$sha"
if [ "$size" -eq "$TOTAL" ] && [ "$sha" = "$SHA_EXPECT" ]; then
  echo "DOWNLOAD_VERIFIED_OK"
else
  echo "DOWNLOAD_VERIFY_FAILED"
  exit 2
fi
