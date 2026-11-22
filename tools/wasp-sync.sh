#!/usr/bin/env bash
# Copy the newest downloaded WASP files into this repo, commit and push.
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
grab 'wasp-requirements*.sysml'  00-requirements/wasp-requirements.sysml           'package WASP'
grab 'wasp_sim*.py'              01-simulation/wasp_sim.py                         'physics model and altitude step test'
grab 'wasp_sim*.m'               01-simulation/wasp_sim.m                          'physics model and altitude step test'
grab 'wasp_attitude*.py'         01-simulation/wasp_attitude.py                    'attitude (roll and pitch) model'
grab 'wasp_attitude*.m'          01-simulation/wasp_attitude.m                     'roll / pitch (attitude) model'
grab 'wasp_link*.py'             04-tracker/gesture-link/wasp_link.py              'WASP Gesture Link'
# keep the integrator limit fix (80 -> 300) if an older copy was downloaded again
sed -i.bak -e 's/-80), 80);/-300), 300);/' 01-simulation/wasp_attitude.m 2>/dev/null; rm -f 01-simulation/wasp_attitude.m.bak
sed -i.bak -e 's/-80, 80)/-300, 300)/' 01-simulation/wasp_attitude.py 2>/dev/null; rm -f 01-simulation/wasp_attitude.py.bak
git add -A
if git diff --cached --quiet; then echo "Nothing changed, nothing to commit."; exit 0; fi
git commit -q -m "${1:-Update WASP files}" && echo "Committed: ${1:-Update WASP files}"
if [ "${NO_PUSH:-0}" = "1" ]; then echo "NO_PUSH=1, not pushing."; exit 0; fi
git push && echo "Pushed." || echo "Push failed. Log in with:  gh auth login   then run:  git push"
