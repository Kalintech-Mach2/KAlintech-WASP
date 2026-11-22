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
