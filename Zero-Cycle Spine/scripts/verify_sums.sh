#!/usr/bin/env bash
# Verify downloaded archives against the committed manifest.
#
#   bash scripts/verify_sums.sh /path/to/ZeroCyclePaper [manifest]
#
# Portable across GNU coreutils (sha256sum) and macOS (shasum -a 256).

set -uo pipefail

DATA_DIR="${1:-}"
MANIFEST="${2:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/SHA256SUMS.txt}"

if [[ -z "$DATA_DIR" ]]; then
    echo "usage: $0 <data-dir> [manifest]" >&2
    exit 2
fi

if [[ ! -d "$DATA_DIR" ]]; then
    echo "error: not a directory: $DATA_DIR" >&2
    exit 2
fi

if [[ ! -f "$MANIFEST" ]]; then
    echo "error: manifest not found: $MANIFEST" >&2
    exit 2
fi

MANIFEST="$(cd "$(dirname "$MANIFEST")" && pwd)/$(basename "$MANIFEST")"

if command -v sha256sum >/dev/null 2>&1; then
    CHECK=(sha256sum -c --quiet)
    CHECK_VERBOSE=(sha256sum -c)
elif command -v shasum >/dev/null 2>&1; then
    CHECK=(shasum -a 256 -c -s)
    CHECK_VERBOSE=(shasum -a 256 -c)
else
    echo "error: neither sha256sum nor shasum found" >&2
    exit 2
fi

EXPECTED=$(grep -cve '^[[:space:]]*$' -e '^[[:space:]]*#' "$MANIFEST" || true)

if [[ "$EXPECTED" -eq 0 ]]; then
    echo "error: manifest contains no checksum lines: $MANIFEST" >&2
    echo "       regenerate it with scripts/make_sums.ps1" >&2
    exit 2
fi
echo "Verifying $EXPECTED file(s) in $DATA_DIR"
echo "against $MANIFEST"
echo

cd "$DATA_DIR" || exit 2

if "${CHECK[@]}" "$MANIFEST"; then
    echo "OK — all $EXPECTED file(s) match."
    exit 0
fi

echo
echo "MISMATCH. Detail follows; lines marked FAILED need re-downloading." >&2
echo >&2
"${CHECK_VERBOSE[@]}" "$MANIFEST" 2>&1 | grep -v ': OK$' >&2 || true
exit 1
