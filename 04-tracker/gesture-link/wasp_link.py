#!/usr/bin/env python3
"""
WASP Gesture Link - relay + simulated drone.  Python 3.8+, standard library only.

  python3 wasp_link.py                  # serve the page + simulated drone on http://127.0.0.1:8000
  python3 wasp_link.py --udp 192.168.4.1:4210   # also forward every valid frame to the real drone

The browser page (index.html) does the camera + gesture recognition and POSTs
two-byte frames [code, code ^ 0xFF] to /api/frame. This script checks each frame,
runs it through a simulated drone (same rules as the real firmware must follow),
and optionally forwards it over UDP.

Safety rules the simulator enforces (mirrors requirements R5/R8):
  * Frames whose second byte is not the inverse of the first are rejected.
  * ARM only from DISARMED, on the ground.  TAKEOFF only when ARMED.
  * Heartbeat (0x01) expected at 10 Hz.  No heartbeat for 0.5 s while armed or
    airborne -> FAILSAFE: motors off (it does NOT try to land itself).
  * KILL (0xE0) works in any state: motors off immediately.
  * After landing the drone disarms itself.
"""
import argparse, glob, json, math, os, socket, sys, threading, time, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))

CODES = {
    0x01: "HEARTBEAT", 0x02: "ARM", 0x03: "DISARM",
    0x10: "TAKEOFF", 0x11: "LAND", 0x20: "HOVER_LOW",
    0x30: "NUDGE_FWD", 0x31: "NUDGE_BACK", 0x40: "YAW_360",
    0x50: "FOLLOW_ON", 0x51: "FOLLOW_OFF", 0xE0: "KILL",
}
HEARTBEAT_TIMEOUT = 0.5   # s
TAKEOFF_ALT = 0.5         # m
LOW_ALT = 0.10            # m
NUDGE_STEP = 0.15         # m
YAW_RATE = 180.0          # deg/s
LAND_RATE = 0.3           # m/s
G = 9.81


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Drone:
    """Very small altitude/x/yaw model. step(dt) is called by a thread (or by tests)."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.reset()

    def reset(self):
        with self.lock:
            self.mode = "DISARMED"
            self.z = 0.0; self.vz = 0.0; self.zt = 0.0
            self.x = 0.0; self.xt = 0.0
            self.yaw = 0.0; self.yaw_left = 0.0
            self.following = False
            self.motors = False
            self.last_hb = None          # time of last heartbeat, None = never heard
            self.hb_count = 0; self.frame_count = 0; self.bad_count = 0
            self.log = []
            self.crashed = False
            self.t0 = self.clock()
            self.udp = getattr(self, 'udp', None)

    # ---- helpers -------------------------------------------------------
    def _note(self, msg):
        self.log.append({"t": round(self.clock() - self.t0, 2), "msg": msg})
        if len(self.log) > 60:
            self.log = self.log[-60:]

    @property
    def airborne(self):
        return self.mode in ("TAKEOFF", "HOVER", "LANDING") and self.z > 0.02

    def _motors_off(self, mode):
        self.mode = mode; self.motors = False
        self.following = False; self.yaw_left = 0.0

    # ---- frame handling ------------------------------------------------
    def frame(self, data):
        """data: bytes. Returns (ok, name, result)."""
        with self.lock:
            if len(data) != 2 or data[1] != (data[0] ^ 0xFF):
                self.bad_count += 1
                self._note("REJECTED bad frame %s" % data.hex())
                return False, None, "rejected: bad checksum"
            code = data[0]
            name = CODES.get(code)
            if name is None:
                self.bad_count += 1
                self._note("REJECTED unknown code 0x%02X" % code)
                return False, None, "rejected: unknown code"
            if name == "HEARTBEAT":
                self.last_hb = self.clock(); self.hb_count += 1
                return True, name, "heartbeat"
            self.frame_count += 1
            ok, why = self._apply(name)
            self._note("%s -> %s" % (name, "ok" if ok else "ignored (%s)" % why))
            return True, name, "ok" if ok else "ignored: " + why

    def _apply(self, name):
        m = self.mode
        if name == "KILL":
            self._motors_off("KILLED"); return True, ""
        if name == "ARM":
            if m != "DISARMED": return False, "not disarmed"
            if self.last_hb is None: return False, "no heartbeat yet"
            self.mode = "ARMED"; self.crashed = False; return True, ""
        if name == "DISARM":
            if m in ("DISARMED",): return False, "already disarmed"
            if self.z > 0.02 and m not in ("KILLED", "FAILSAFE"):
                return False, "airborne"
            self.mode = "DISARMED"; self.motors = False; return True, ""
        if name == "TAKEOFF":
            if m != "ARMED": return False, "not armed"
            self.mode = "TAKEOFF"; self.motors = True; self.zt = TAKEOFF_ALT; return True, ""
        if name == "LAND":
            if m not in ("TAKEOFF", "HOVER"): return False, "not airborne"
            self.mode = "LANDING"; self.yaw_left = 0.0; self.following = False; return True, ""
        if m not in ("HOVER", "TAKEOFF"):
            return False, "not airborne"
        if name == "HOVER_LOW":
            self.zt = LOW_ALT; self.mode = "HOVER"; return True, ""
        if m != "HOVER":
            return False, "still climbing"
        if self.yaw_left > 0 and name in ("NUDGE_FWD", "NUDGE_BACK", "YAW_360"):
            return False, "busy turning"
        if name == "NUDGE_FWD":  self.xt += NUDGE_STEP; return True, ""
        if name == "NUDGE_BACK": self.xt -= NUDGE_STEP; return True, ""
        if name == "YAW_360":    self.yaw_left = 360.0; return True, ""
        if name == "FOLLOW_ON":  self.following = True; return True, ""
        if name == "FOLLOW_OFF": self.following = False; return True, ""
        return False, "unhandled"

    # ---- physics -------------------------------------------------------
    def step(self, dt):
        with self.lock:
            now = self.clock()
            # heartbeat failsafe (R5): motors off, not "land"
            if (self.mode in ("ARMED", "TAKEOFF", "HOVER", "LANDING") and self.last_hb is not None
                    and now - self.last_hb > HEARTBEAT_TIMEOUT):
                if self.z > 0.02:
                    self._motors_off("FAILSAFE")
                    self._note("HEARTBEAT LOST -> motors off")
                else:
                    self.mode = "DISARMED"; self.motors = False
                    self._note("HEARTBEAT LOST -> disarmed")
            if self.motors and self.mode in ("TAKEOFF", "HOVER", "LANDING"):
                if self.mode == "LANDING":
                    self.zt = max(0.0, self.z - LAND_RATE * dt * 3) if self.z > 0.05 else 0.0
                    vcmd = -LAND_RATE
                else:
                    vcmd = clamp(3.0 * (self.zt - self.z), -1.5, 1.5)
                self.vz += (vcmd - self.vz) * min(1.0, dt / 0.15)
                self.z += self.vz * dt
                self.x += (self.xt - self.x) * min(1.0, dt / 0.3)
                if self.yaw_left > 0:
                    d = min(self.yaw_left, YAW_RATE * dt)
                    self.yaw = (self.yaw + d) % 360.0; self.yaw_left -= d
                if self.mode == "TAKEOFF" and abs(self.z - self.zt) < 0.03 and abs(self.vz) < 0.1:
                    self.mode = "HOVER"
                if self.mode == "LANDING" and self.z <= 0.02:
                    self.z = 0.0; self.vz = 0.0; self.motors = False
                    self.mode = "DISARMED"; self.xt = self.x
                    self._note("landed, disarmed")
            else:
                # motors off: free fall
                if self.z > 0.0 or self.vz != 0.0:
                    self.vz -= G * dt; self.z += self.vz * dt
                    if self.z <= 0.0:
                        if self.vz < -1.5:
                            self.crashed = True
                            self._note("IMPACT %.1f m/s" % -self.vz)
                        self.z = 0.0; self.vz = 0.0

    def state(self):
        with self.lock:
            now = self.clock()
            age = None if self.last_hb is None else round(now - self.last_hb, 2)
            return {
                "mode": self.mode, "z": round(self.z, 3), "vz": round(self.vz, 3),
                "x": round(self.x, 3), "yaw": round(self.yaw, 1),
                "yaw_left": round(self.yaw_left, 1), "following": self.following,
                "motors": self.motors, "crashed": self.crashed,
                "hb_age": age, "link_ok": age is not None and age <= HEARTBEAT_TIMEOUT,
                "hb_count": self.hb_count, "frame_count": self.frame_count,
                "bad_count": self.bad_count, "udp": self.udp, "log": self.log[-25:],
            }


# ---------------------------------------------------------------------------
def newest(patterns):
    files = []
    for p in patterns:
        files += glob.glob(os.path.expanduser(p))
    files = [f for f in files if os.path.isfile(f)]
    return max(files, key=os.path.getmtime) if files else None


def make_handler(drone, args, udp):
    class H(BaseHTTPRequestHandler):
        server_version = "WASPLink/1"

        def log_message(self, fmt, *a):
            if args.verbose:
                sys.stderr.write("[http] " + fmt % a + "\n")

        def _send(self, code, body, ctype="application/json"):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                f = os.path.join(HERE, "index.html")
                if not os.path.exists(f):
                    f = newest([os.path.join(HERE, "wasp-link*.html"), "~/Downloads/wasp-link*.html"])
                if not f or not os.path.exists(f):
                    return self._send(404, "Gesture Link page not found: put index.html (or wasp-link.html) next to wasp_link.py", "text/plain")
                with open(f, "rb") as fh:
                    return self._send(200, fh.read(), "text/html; charset=utf-8")
            if path == "/api/state":
                return self._send(200, drone.state())
            if path == "/api/command-table":
                cands = [args.table] if args.table else [
                    os.path.join(HERE, "..", "gesture-studio", "wasp-command-table.json"),
                    newest([os.path.join(HERE, "wasp-command-table*.json"), "~/Downloads/wasp-command-table*.json"])]
                for c in cands:
                    if c and os.path.exists(c):
                        with open(c, "rb") as fh:
                            return self._send(200, fh.read())
                return self._send(404, {"error": "command table not found; use --table"})
            if path == "/api/gestures":
                f = args.gestures or newest(["~/Downloads/wasp-gestures*.json",
                                             os.path.join(HERE, "..", "gesture-studio", "wasp-gestures*.json")])
                if not f or not os.path.exists(f):
                    return self._send(404, {"error": "no wasp-gestures*.json found; use --gestures or pick the file in the page"})
                with open(f, "rb") as fh:
                    data = fh.read()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("X-Source-File", os.path.basename(f))
                self.send_header("Access-Control-Expose-Headers", "X-Source-File")
                self.end_headers(); self.wfile.write(data); return
            self._send(404, {"error": "not found"})

        def do_POST(self):
            path = self.path.split("?")[0]
            n = int(self.headers.get("Content-Length") or 0)
            body = self.rfile.read(n) if n else b""
            if path == "/api/frame":
                ok, name, result = drone.frame(body)
                if ok and udp:
                    try: udp[0].sendto(body, udp[1])
                    except OSError as e: sys.stderr.write("udp error: %s\n" % e)
                if ok and name != "HEARTBEAT":
                    print("[frame] %s -> %s" % (name, result), flush=True)
                return self._send(200 if ok else 400,
                                  {"ok": ok, "name": name, "result": result, "state": drone.state()})
            if path == "/api/reset":
                drone.reset(); return self._send(200, {"ok": True})
            self._send(404, {"error": "not found"})
    return H


def run_sim(drone, stop):
    last = time.monotonic()
    while not stop.is_set():
        time.sleep(0.01)
        now = time.monotonic()
        drone.step(min(0.05, now - last)); last = now


def main(argv=None):
    ap = argparse.ArgumentParser(description="WASP Gesture Link relay + simulated drone")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--table", help="command table JSON (default: ../gesture-studio/wasp-command-table.json)")
    ap.add_argument("--gestures", help="gestures export JSON (default: newest ~/Downloads/wasp-gestures*.json)")
    ap.add_argument("--udp", help="also forward valid frames to host:port (real drone)")
    ap.add_argument("--no-open", action="store_true", help="do not open the browser")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    udp = None
    if args.udp:
        host, _, port = args.udp.rpartition(":")
        udp = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (host, int(port)))
        print("forwarding frames to %s:%s" % (host, port))

    drone = Drone()
    drone.udp = args.udp
    stop = threading.Event()
    threading.Thread(target=run_sim, args=(drone, stop), daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(drone, args, udp))
    url = "http://127.0.0.1:%d/" % args.port
    print("WASP Gesture Link on %s   (Ctrl+C to stop)" % url)
    if not args.no_open:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        stop.set(); srv.server_close()


if __name__ == "__main__":
    main()
