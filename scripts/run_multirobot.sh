#!/usr/bin/env bash
# Run manually in the NUC desktop terminal; DISPLAY/XAUTHORITY stay with that session.
set -euo pipefail
SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
usage() {
  echo "Usage: bash scripts/run_multirobot.sh [infantry|sentry] [own] [target]"
  echo "       bash scripts/run_multirobot.sh --config <project-relative-or-absolute.toml>"
  echo "own: standard | wheel_leg | low | folding"
  echo "target: standard | wheel_leg | low | folding | heavy_target"
}
TEMP_CONFIG=""
cleanup() { [[ -z "$TEMP_CONFIG" ]] || rm -f -- "$TEMP_CONFIG"; }
trap cleanup EXIT
case "${1:-}" in
  -h|--help) usage; exit 0 ;;
  --config)
    [[ $# -eq 2 ]] || { usage >&2; exit 2; }
    CONFIG="$2"
    [[ "$CONFIG" = /* ]] || CONFIG="$SIM_ROOT/$CONFIG"
    [[ -f "$CONFIG" ]] || { echo "Config not found: $CONFIG" >&2; exit 2; }
    ;;
  *)
    [[ $# -le 3 ]] || { usage >&2; exit 2; }
    ROLE="${1:-infantry}"
    OWN="${2:-standard}"
    TARGET="${3:-standard}"
    case "$ROLE" in infantry|sentry) ;; *) usage >&2; exit 2 ;; esac
    case "$OWN" in standard|wheel_leg|low|folding) ;; *) usage >&2; exit 2 ;; esac
    case "$TARGET" in standard|wheel_leg|low|folding|heavy_target) ;; *) usage >&2; exit 2 ;; esac
    CONFIG=""
    ;;
esac
SIM_BIN="$SIM_ROOT/target/release/daedalus"
[[ -x "$SIM_BIN" ]] || { echo "Build daedalus first (cargo build --release --bin daedalus -j 2)." >&2; exit 1; }
RUSTC_BIN="${RUSTC_BIN:-rustc}"
if ! command -v "$RUSTC_BIN" >/dev/null 2>&1; then
  RUSTC_BIN="$HOME/.cargo/bin/rustc"
fi
if [[ -z "$CONFIG" ]]; then
  TEMP_CONFIG="$(mktemp /tmp/rmul-multirobot-XXXXXX.toml)"
  CONFIG="$TEMP_CONFIG"
  python3 -B - "$SIM_ROOT" "$ROLE" "$OWN" "$TARGET" "$CONFIG" <<'PY'
from pathlib import Path
import sys
root, role, own, target, out = sys.argv[1:]
text = (Path(root) / 'configs' / f'rmul_2027_multirobot_{role}.toml').read_text()
text = text[:text.index('[robots]')] + f'[robots]\ncontrolled = "{own}"\ntarget = "{target}"\n'
Path(out).write_text(text)
PY
fi
export BEVY_ASSET_ROOT="$SIM_ROOT"
export LD_LIBRARY_PATH="$SIM_ROOT/target/release/deps:$("$RUSTC_BIN" --print target-libdir)${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
echo "Config: $CONFIG (edits apply on R; map requires restart)"
cd -- "$SIM_ROOT"
"$SIM_BIN" --config "$CONFIG"
