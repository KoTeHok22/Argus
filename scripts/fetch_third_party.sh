#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p third_party
DEST=third_party/patchworkpp
SHA=3e6903a1d5537a4cc2ace897b0bbb98a92d6014c
URL=https://github.com/url-kaist/patchwork-plusplus.git
if [ -d "$DEST/.git" ]; then
  git -C "$DEST" fetch origin "$SHA" || git -C "$DEST" fetch origin
  git -C "$DEST" checkout --detach "$SHA"
else
  git clone "$URL" "$DEST"
  git -C "$DEST" checkout --detach "$SHA"
fi
echo "OK: $DEST @ $(git -C "$DEST" rev-parse --short HEAD)"
