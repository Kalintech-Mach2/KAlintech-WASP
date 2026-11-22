#!/usr/bin/env bash
# Copy the newest downloaded WASP files into this repo, commit and push.   (v2)
#   tools/wasp-sync.sh "message"
#   NO_PUSH=1 tools/wasp-sync.sh        commit only
#   WASP_DOWNLOADS=/path                if you do not download to ~/Downloads
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
DL="${WASP_DOWNLOADS:-$HOME/Downloads}"
cd "$REPO" || exit 1
grab() { # glob  destination  text-that-must-be-inside
  local f="" x
  while IFS= read -r x; do
    if grep -q -- "$3" "$x" 2>/dev/null; then f="$x"; break; fi
  done < <(ls -t "$DL"/$1 2>/dev/null)
  if [ -n "$f" ]; then
    if [ -f "$2" ] && cmp -s "$f" "$2"; then echo "  same     $2"
    else mkdir -p "$(dirname "$2")"; cp "$f" "$2"; echo "  updated  $2   <- $(basename "$f")"; fi
  fi
}
echo "Looking in $DL"
grab 'index*.html'               04-tracker/gesture-studio/index.html              'WASP Gesture Studio'
grab 'wasp-command-table*.json'  04-tracker/gesture-studio/wasp-command-table.json 'WASP command table'
grab 'wasp-link*.html'           04-tracker/gesture-link/index.html                'WASP Gesture Link'
grab 'wasp_link*.py'             04-tracker/gesture-link/wasp_link.py              'WASP Gesture Link - relay'
grab 'wasp_link_test*.py'        04-tracker/gesture-link/test_link.py              'import wasp_link'
grab 'wasp-requirements*.sysml'  00-requirements/wasp-requirements.sysml           'package WASP'
grab 'wasp_sim*.py'              01-simulation/wasp_sim.py                         'physics model and altitude step test'
grab 'wasp_sim*.m'               01-simulation/wasp_sim.m                          'physics model and altitude step test'
grab 'wasp_attitude*.py'         01-simulation/wasp_attitude.py                    'attitude (roll and pitch) model'
grab 'wasp_attitude*.m'          01-simulation/wasp_attitude.m                     'roll / pitch (attitude) model'
# the Link also reads the table from its own folder, so keep one copy beside it
if [ -f 04-tracker/gesture-studio/wasp-command-table.json ] && [ -d 04-tracker/gesture-link ]; then
  cp 04-tracker/gesture-studio/wasp-command-table.json 04-tracker/gesture-link/wasp-command-table.json
fi
# README for the Link, written once
if [ -d 04-tracker/gesture-link ] && [ ! -f 04-tracker/gesture-link/README.md ]; then
cat > 04-tracker/gesture-link/README.md <<'MD'
# WASP Gesture Link

Turns hand and body gestures into two-byte commands and sends them to a simulated drone
(later the real one).

    python3 04-tracker/gesture-link/wasp_link.py        # opens http://127.0.0.1:8000
    python3 04-tracker/gesture-link/test_link.py        # relay + simulator tests

- `wasp_link.py` relay + simulated drone (standard library only)
- `index.html` the page: camera, recognition, per-command confidence gates, arm and kill
- `wasp-command-table.json` gesture -> code table (see `frame_format` and `rules` inside)

Frame: `[code, code XOR 0xFF]`, heartbeat 0x01 at 10 Hz, no heartbeat for 0.5 s = motors off.
Add `--udp host:port` to also forward frames to the real drone. Gesture recordings stay local
(`wasp-gestures*.json` is in .gitignore).
MD
echo "  wrote    04-tracker/gesture-link/README.md"
fi
# never commit Python caches or private gesture recordings
for pat in '__pycache__/' '*.pyc' 'wasp-gestures*.json' '.DS_Store'; do
  grep -qxF "$pat" .gitignore 2>/dev/null || echo "$pat" >> .gitignore
done
# keep the integrator limit fix (80 -> 300) if an older copy was downloaded again
sed -i.bak -e 's/-80), 80);/-300), 300);/' 01-simulation/wasp_attitude.m 2>/dev/null; rm -f 01-simulation/wasp_attitude.m.bak
sed -i.bak -e 's/-80, 80)/-300, 300)/' 01-simulation/wasp_attitude.py 2>/dev/null; rm -f 01-simulation/wasp_attitude.py.bak
# update this script itself from a newer download (safe: copy then move, takes effect next run)
new="$(ls -t "$DL"/wasp-sync*.sh 2>/dev/null | head -1)"
if [ -n "$new" ] && grep -q 'wasp-sync' "$new" && ! cmp -s "$new" tools/wasp-sync.sh; then
  cp "$new" tools/wasp-sync.sh.new && chmod +x tools/wasp-sync.sh.new && mv tools/wasp-sync.sh.new tools/wasp-sync.sh
  echo "  updated  tools/wasp-sync.sh (new version used from the next run)"
fi
git add -A
if git diff --cached --quiet; then echo "Nothing changed, nothing to commit."; exit 0; fi
git commit -q -m "${1:-Update WASP files}" && echo "Committed: ${1:-Update WASP files}"
if [ "${NO_PUSH:-0}" = "1" ]; then echo "NO_PUSH=1, not pushing."; exit 0; fi
git push && echo "Pushed." || echo "Push failed. Log in with:  gh auth login   then run:  git push"
