# Gesture Studio

A single web page that records, names and exports hand and body gestures using MediaPipe.

    python3 -m http.server 8000

Open http://localhost:8000 in Chrome and press Start camera. Record each gesture, then Export JSON.

- `index.html`: the Studio. Hand mode (up to two hands) and Body mode (upper body).
- `wasp-command-table.json`: which gesture sends which two-byte command, with hold times and
  minimum confidence per command.

Your own exported recordings (`wasp-gestures-*.json`) are ignored by git on purpose.
