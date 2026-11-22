import sys, unittest, json, threading, urllib.request, time
sys.path.insert(0, '.')
import wasp_link as W

class Clk:
    t = 0.0
    def __call__(self): return self.t

def fr(code): return bytes([code, code ^ 0xFF])

class T(unittest.TestCase):
    def setUp(self):
        self.c = Clk(); self.d = W.Drone(self.c)
    def run_for(self, secs, hb=True):
        n = int(secs / 0.01)
        for i in range(n):
            self.c.t += 0.01
            if hb and i % 10 == 0: self.d.frame(fr(1))
            self.d.step(0.01)
    def arm(self):
        self.d.frame(fr(1)); self.assertEqual(self.d.frame(fr(2))[2], "ok"); self.assertEqual(self.d.mode, "ARMED")
    def fly(self):
        self.arm(); self.d.frame(fr(0x10)); self.run_for(3); self.assertEqual(self.d.mode, "HOVER")

    def test_bad_frames(self):
        self.assertFalse(self.d.frame(bytes([0x10, 0x10]))[0])
        self.assertFalse(self.d.frame(b'\x10')[0])
        self.assertFalse(self.d.frame(fr(0x77))[0])
        self.assertEqual(self.d.bad_count, 3)
    def test_arm_needs_heartbeat(self):
        self.assertIn("no heartbeat", self.d.frame(fr(2))[2])
    def test_takeoff_needs_arm(self):
        self.d.frame(fr(1))
        self.assertIn("not armed", self.d.frame(fr(0x10))[2]); self.assertEqual(self.d.mode, "DISARMED")
    def test_takeoff_hover_alt(self):
        self.fly(); self.assertAlmostEqual(self.d.z, 0.5, delta=0.03)
    def test_settle_time(self):
        self.arm(); self.d.frame(fr(0x10)); t = 0
        while self.d.mode != "HOVER" and t < 5: self.run_for(0.01); t += 0.01
        print("takeoff settled in %.2fs" % t); self.assertLess(t, 3)
    def test_hover_low(self):
        self.fly(); self.d.frame(fr(0x20)); self.run_for(3); self.assertAlmostEqual(self.d.z, 0.10, delta=0.03)
    def test_nudges(self):
        self.fly(); self.d.frame(fr(0x30)); self.run_for(2); self.assertAlmostEqual(self.d.x, 0.15, delta=0.02)
        self.d.frame(fr(0x31)); self.d.frame(fr(0x31)); self.run_for(2); self.assertAlmostEqual(self.d.x, -0.15, delta=0.02)
    def test_nudge_ignored_grounded(self):
        self.arm(); self.assertIn("not airborne", self.d.frame(fr(0x30))[2]); self.assertEqual(self.d.xt, 0)
    def test_yaw(self):
        self.fly(); y0 = self.d.yaw; self.d.frame(fr(0x40)); self.run_for(1)
        self.assertIn("busy", self.d.frame(fr(0x30))[2]); self.run_for(1.5)
        self.assertAlmostEqual(self.d.yaw, y0, delta=1.0); self.assertEqual(self.d.yaw_left, 0)
    def test_follow(self):
        self.fly(); self.d.frame(fr(0x50)); self.assertTrue(self.d.following)
        self.d.frame(fr(0x51)); self.assertFalse(self.d.following)
    def test_land_disarms(self):
        self.fly(); self.d.frame(fr(0x11)); self.run_for(4)
        self.assertEqual(self.d.mode, "DISARMED"); self.assertEqual(self.d.z, 0); self.assertFalse(self.d.crashed)
    def test_kill_freefall(self):
        self.fly(); self.d.frame(fr(0xE0)); self.assertEqual(self.d.mode, "KILLED"); self.assertFalse(self.d.motors)
        self.run_for(1); self.assertEqual(self.d.z, 0); self.assertTrue(self.d.state()["mode"] == "KILLED")
        self.assertIn("ignored", self.d.frame(fr(0x10))[2])
        self.assertEqual(self.d.frame(fr(3))[2], "ok"); self.assertEqual(self.d.mode, "DISARMED")
    def test_kill_works_anywhere(self):
        self.assertEqual(self.d.frame(fr(0xE0))[2], "ok"); self.assertEqual(self.d.mode, "KILLED")
    def test_failsafe_stages(self):
        self.fly(); z0 = self.d.z
        self.run_for(0.2, hb=False); self.assertEqual(self.d.mode, "HOVER")          # still fine at 0.2 s
        self.run_for(0.25, hb=False); self.assertEqual(self.d.mode, "LINK_HOLD")     # > 0.3 s
        self.assertTrue(self.d.motors); self.assertAlmostEqual(self.d.z, z0, delta=0.03)
        self.run_for(0.8, hb=False); self.assertEqual(self.d.mode, "LINK_DESCENT")   # > 1.0 s
        z1 = self.d.z; self.run_for(0.5, hb=False); self.assertLess(self.d.z, z1 - 0.05)
        self.assertGreater(self.d.z, 0.0)
        self.run_for(1.6, hb=False)                                                  # > 3.0 s
        self.assertEqual(self.d.mode, "FAILSAFE"); self.assertFalse(self.d.motors)
    def test_short_dropout_recovers(self):
        self.fly(); self.run_for(0.6, hb=False); self.assertEqual(self.d.mode, "LINK_HOLD")
        self.run_for(0.3); self.assertEqual(self.d.mode, "HOVER"); self.assertTrue(self.d.motors)
    def test_descent_then_link_back_lands(self):
        self.fly(); self.run_for(1.3, hb=False); self.assertEqual(self.d.mode, "LINK_DESCENT")
        self.run_for(0.3); self.assertEqual(self.d.mode, "LANDING"); self.run_for(4)
        self.assertEqual(self.d.mode, "DISARMED"); self.assertFalse(self.d.crashed)
    def test_failsafe_timing(self):
        self.fly(); t = 0
        while self.d.mode == "HOVER" and t < 2: self.run_for(0.01, hb=False); t += 0.01
        self.assertAlmostEqual(t, 0.3, delta=0.12)
    def test_grounded_armed_hb_loss_disarms(self):
        self.arm(); self.run_for(1.3, hb=False); self.assertEqual(self.d.mode, "DISARMED")
    def test_kill_in_link_hold(self):
        self.fly(); self.run_for(0.6, hb=False); self.d.frame(fr(1)); self.assertEqual(self.d.frame(fr(0xE0))[2], "ok")
        self.assertEqual(self.d.mode, "KILLED"); self.assertFalse(self.d.motors)
    def test_disarm_blocked_airborne(self):
        self.fly(); self.assertIn("airborne", self.d.frame(fr(3))[2])
    def test_double_takeoff_ignored(self):
        self.fly(); self.assertIn("not armed", self.d.frame(fr(0x10))[2])

class HTTP(unittest.TestCase):
    def test_server(self):
        import argparse
        a = argparse.Namespace(table=None, gestures=None, verbose=False)
        d = W.Drone(); stop = threading.Event()
        threading.Thread(target=W.run_sim, args=(d, stop), daemon=True).start()
        srv = W.ThreadingHTTPServer(("127.0.0.1", 0), W.make_handler(d, a, None))
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        base = "http://127.0.0.1:%d" % srv.server_port
        def post(b):
            try:
                r = urllib.request.urlopen(urllib.request.Request(base + "/api/frame", data=b, method="POST"))
                return r.status, json.loads(r.read())
            except urllib.error.HTTPError as e: return e.code, json.loads(e.read())
        self.assertEqual(post(fr(1))[0], 200)
        s, j = post(fr(2)); self.assertEqual(j["state"]["mode"], "ARMED")
        self.assertEqual(post(b'\x10\x10')[0], 400)
        post(fr(0x10))
        for _ in range(11): time.sleep(0.2); post(fr(1))
        st = json.loads(urllib.request.urlopen(base + "/api/state").read()); print("http state", st["mode"], st["z"])
        self.assertIn(st["mode"], ("TAKEOFF", "HOVER")); self.assertGreater(st["z"], 0.3)
        post(fr(0xE0)); time.sleep(0.3)
        self.assertEqual(json.loads(urllib.request.urlopen(base + "/api/state").read())["mode"], "KILLED")
        stop.set(); srv.shutdown()

if __name__ == "__main__": unittest.main(verbosity=1)
