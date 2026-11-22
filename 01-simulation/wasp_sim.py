"""
WASP Phase 1 - physics model and altitude step test.

What this file does
  1. Weight budget            (section 1)
  2. Motor + prop model       (section 2)  YSIDO 1505 3750KV, 3S, Gemfan 3016
  3. Altitude step test       (section 3+)  0 m -> 1 m, settle within +-2 cm

Run:  python3 wasp_sim.py
Needs: numpy, scipy, matplotlib

Honest labels used in comments
  SELLER   = number from the seller / datasheet
  ASSUMED  = my estimate, check it when the parts arrive
The biggest unknown is the prop thrust coefficient Ct. Everything that
depends on it is shown as a range, and the thrust stand test will pin it down.
"""
import numpy as np
from scipy.optimize import brentq

RHO = 1.225   # air density, kg/m^3
G = 9.81      # gravity, m/s^2

# ---------------------------------------------------------------------------
# 1. WEIGHT BUDGET (grams)
# ---------------------------------------------------------------------------
BUDGET = [
    ("Motors x4 (11 g each)",            44.0, "SELLER"),
    ("4-in-1 ESC (JHEMCU EM40A)",         5.2, "SELLER"),
    ("Battery 3S 650 mAh Tattu",         59.0, "SELLER, +-10 to 20 g tolerance"),
    ("Props x4 (Gemfan 3016, ~1.2 g)",    4.8, "ASSUMED"),
    ("ESP32-S3 SuperMini",                2.5, "ASSUMED"),
    ("MPU-6050 GY-521",                   2.1, "ASSUMED"),
    ("5 V buck regulator",                1.3, "ASSUMED"),
    ("Custom PCB + headers",              6.0, "ASSUMED"),
    ("Wiring, XT30 pigtail, connectors",  6.0, "ASSUMED"),
    ("Frame (arms + plates), design target", 22.0, "ASSUMED, design to this"),
    ("Screws, standoffs, ties",           5.0, "ASSUMED"),
    ("Battery strap",                     3.0, "ASSUMED"),
    ("Solder, tape, misc",                2.0, "ASSUMED"),
    ("Laser ToF sensor (VL53L1X)",        2.0, "ASSUMED"),
]
MASS_NOMINAL = sum(m for _, m, _ in BUDGET) / 1000.0   # kg


# ---------------------------------------------------------------------------
# 2. MOTOR + PROP MODEL
# ---------------------------------------------------------------------------
class Nominal:
    D = 0.0762            # prop diameter, m (3 in)
    Kv = 3750.0           # rpm per volt                      SELLER
    Rm = 0.096 + 0.03     # ohm: motor 96 mOhm (SELLER) + ESC FET/wire ASSUMED
    I0 = 0.6              # A, no-load / friction current     ASSUMED
    Ct = 0.12             # thrust coeff, T = Ct*rho*n^2*D^4  ASSUMED (main unknown)
    FoM = 0.44            # figure of merit, calibrated on seller test:
                          # 631 g @ 288 W with a 4 in prop -> FoM about 0.44
    J = 9e-7              # kg m^2, rotor + prop inertia      ASSUMED
    Rb = 0.04             # ohm, battery internal resistance, 3S 650 mAh ASSUMED
    Voc = 12.0            # V, charged pack under light load
    CdA = 0.0144          # m^2, drag area for vertical motion ASSUMED
    mass = MASS_NOMINAL


def cp_from_ct(Ct, FoM):
    """Momentum theory links power and thrust coefficients: Cp = 0.798*Ct^1.5/FoM"""
    return 0.7979 * Ct ** 1.5 / FoM


def motor_consts(Kv):
    Ke = 60.0 / (2 * np.pi * Kv)     # V per rad/s  (same number as N.m per A)
    return Ke


def steady_state(duty, Voc, Ct=Nominal.Ct, Kv=Nominal.Kv, n_motors=4):
    """Steady rotor speed, thrust and current at a given ESC duty (0..1)."""
    p = Nominal
    Ke = motor_consts(Kv)
    Cp = cp_from_ct(Ct, p.FoM)
    Cq = Cp / (2 * np.pi)

    def f(w):
        n = w / (2 * np.pi)
        Q = Cq * RHO * n ** 2 * p.D ** 5
        I = Q / Ke + p.I0
        Vb = Voc - p.Rb * n_motors * I
        return duty * Vb - (Ke * w + I * p.Rm)

    w = brentq(f, 1.0, 2 * np.pi * Kv * Voc / 60.0)
    n = w / (2 * np.pi)
    Q = Cq * RHO * n ** 2 * p.D ** 5
    I = Q / Ke + p.I0
    T = Ct * RHO * n ** 2 * p.D ** 4
    Vb = Voc - p.Rb * n_motors * I
    return dict(w=w, rpm=60 * n, T_N=T, T_g=T / G * 1000, I=I,
                P_in=Vb * I, Vb=Vb, I_total=n_motors * I)


def hover_point(Ct, mass, Voc=11.7):
    """Duty and current needed to hover."""
    T_need = mass * G / 4.0
    duty = brentq(lambda d: steady_state(d, Voc, Ct)["T_N"] - T_need, 0.01, 1.0)
    return duty, steady_state(duty, Voc, Ct)


# ---------------------------------------------------------------------------
# 3. ALTITUDE SIMULATION (vectorised: N runs at once)
# ---------------------------------------------------------------------------
SENSORS = {
    # name: rate Hz, noise sigma m, delay s, random-walk m/sqrt(s), estimator pole rad/s
    "Mocap-grade (ideal)":      dict(fs=100, sigma=0.002, delay=0.010, rw=0.000, w_est=25.0),
    "Laser ToF (VL53-class)":   dict(fs=50,  sigma=0.006, delay=0.020, rw=0.000, w_est=15.0),
    "Barometer (BMP/DPS-class)": dict(fs=25, sigma=0.15,  delay=0.040, rw=0.030, w_est=1.5,
                                      ctrl=dict(Kz=0.8, Kv=8.0, Ki=3.0, vmax=0.6, amax=18.0, a_dec=2.0)),
}

CTRL = dict(Kz=3.0, Kv=12.0, Ki=25.0, vmax=1.5, amax=18.0, a_dec=3.0)   # robust tune (see tune2 sweep)
# Kz   : position loop gain near the target (1/s)
# Kv,Ki: velocity PI loop (accel command per m/s, per m)
# vmax : climb speed limit (m/s)
# a_dec: braking deceleration the position loop plans for (m/s^2)


def simulate(sensor, N=1, T=6.0, dt=1e-3, ctrl_every=2, t_step=0.5,
             z_target=1.0, spread=False, seed=1, ctrl=CTRL, brake=False):
    rng = np.random.default_rng(seed)
    p = Nominal
    nsteps = int(T / dt)
    S = SENSORS[sensor]
    ctrl = S.get("ctrl", ctrl)      # a sensor may bring its own (slower) gains

    # --- per-run "true" parameters (controller only knows the nominal ones)
    def U(lo, hi):
        return rng.uniform(lo, hi, N) if spread else np.full(N, 0.5 * (lo + hi))
    mass = p.mass * U(0.9, 1.1)
    Ct = p.Ct * U(0.8, 1.2)
    Kv = p.Kv * U(0.95, 1.05)
    Voc = U(11.4, 12.6) if spread else np.full(N, p.Voc)
    abias = rng.uniform(-0.05, 0.05, N) if spread else np.zeros(N)

    Ke = motor_consts(Kv)
    Cq = cp_from_ct(Ct, p.FoM) / (2 * np.pi)

    # nominal values used inside the controller
    Ke_n = motor_consts(p.Kv)
    Cq_n = cp_from_ct(p.Ct, p.FoM) / (2 * np.pi)

    # --- states
    z = np.zeros(N); v = np.zeros(N); w = np.zeros(N)
    zh = np.zeros(N); vh = np.zeros(N); bh = np.zeros(N)
    Iv = np.zeros(N); duty = np.zeros(N)
    zm = np.zeros(N); rwalk = np.zeros(N)
    wc = S["w_est"]; k1, k2, k3 = 3 * wc, 3 * wc ** 2, wc ** 3
    dtc = ctrl_every * dt
    sens_every = max(1, int(round(1.0 / S["fs"] / dt)))
    delay_n = int(round(S["delay"] / dt))

    zhist = np.zeros((nsteps + 1, N))
    out = dict(t=np.arange(nsteps) * dt, z=np.zeros((nsteps, N)), duty=np.zeros((nsteps, N)),
               zh=np.zeros((nsteps, N)), I=np.zeros((nsteps, N)))
    Vb = Voc.copy()
    Ib_total = np.zeros(N)

    for k in range(nsteps):
        t = k * dt
        zref = z_target if t >= t_step else 0.0

        # --- physics: motor + prop (all four identical)
        n = w / (2 * np.pi)
        Q = Cq * RHO * n ** 2 * p.D ** 5
        I_raw = (duty * Vb - Ke * w) / p.Rm
        I = I_raw if brake else np.maximum(I_raw, 0.0)     # brake: ESC may brake the rotor
        I = np.maximum(I, -12.0)                            # braking current limit (A)
        Tm = Ct * RHO * n ** 2 * p.D ** 4
        Ib_total = 4 * np.maximum(I, 0.0)   # braking current does not come from the battery
        Vb = Voc - p.Rb * Ib_total
        torque = Ke * (I - p.I0 * (w > 1.0)) - Q
        w = np.maximum(w + dt * torque / p.J, 0.0)

        # --- physics: vertical dynamics
        Fdrag = 0.5 * RHO * p.CdA * v * np.abs(v)
        F = 4 * Tm - mass * G - Fdrag
        on_ground = (z <= 0.0) & (F <= 0.0)
        a = np.where(on_ground, 0.0, F / mass)
        v = np.where(on_ground, 0.0, v + a * dt)
        z = np.where(on_ground, 0.0, np.maximum(z + v * dt, 0.0))
        zhist[k + 1] = z

        # --- sensor sample (position)
        if k % sens_every == 0:
            idx = max(k - delay_n, 0)
            rwalk += S["rw"] * np.sqrt(sens_every * dt) * rng.standard_normal(N)
            zm = zhist[idx] + rwalk + S["sigma"] * rng.standard_normal(N)

        # --- controller + estimator at control rate
        if k % ctrl_every == 0:
            # accelerometer: specific force, noise incl. vibration
            f_spec = np.where(on_ground, G, (4 * Tm - Fdrag) / mass)
            a_meas = f_spec + abias + 0.30 * rng.standard_normal(N) - G
            e = zm - zh
            zh = zh + (vh + k1 * e) * dtc
            vh = vh + (a_meas - bh + k2 * e) * dtc
            bh = bh - k3 * e * dtc

            err = zref - zh
            lin = ctrl["a_dec"] / ctrl["Kz"] ** 2            # below this error: plain P
            vsq = np.sqrt(2 * ctrl["a_dec"] * np.maximum(np.abs(err) - 0.5 * lin, 0.0))
            vcmd = np.where(np.abs(err) <= lin, ctrl["Kz"] * err, np.sign(err) * vsq)
            vcmd = np.clip(vcmd, -ctrl["vmax"], ctrl["vmax"])
            ev = vcmd - vh
            acmd = ctrl["Kv"] * ev + Iv
            acmd_sat = np.clip(acmd, -0.8 * G, ctrl["amax"])
            # anti-windup: only integrate when not saturated
            Iv = np.where(acmd == acmd_sat, Iv + ctrl["Ki"] * ev * dtc, Iv)
            Iv = np.clip(Iv, -6.0, 6.0)

            T_each = np.maximum(p.mass * (G + acmd_sat) / 4.0, 0.0)
            n_c = np.sqrt(T_each / (p.Ct * RHO * p.D ** 4))
            w_c = 2 * np.pi * n_c
            Qc = Cq_n * RHO * n_c ** 2 * p.D ** 5
            Ic = Qc / Ke_n + p.I0
            Vt = Ke_n * w_c + Ic * p.Rm
            duty = np.clip(Vt / Vb, 0.0, 1.0)
            # nothing wanted and still on the ground: motors off
            duty = np.where((zref == 0.0) & (z <= 0.001), 0.0, duty)

        out["z"][k] = z; out["duty"][k] = duty; out["zh"][k] = zh; out["I"][k] = Ib_total
    return out


def metrics(out, t_step=0.5, target=1.0, band=0.02):
    t = out["t"]; z = out["z"]
    res = []
    for i in range(z.shape[1]):
        zi = z[:, i]
        after = t >= t_step
        def first(level):
            idx = np.where(after & (zi >= level))[0]
            return t[idx[0]] - t_step if len(idx) else np.inf
        t10, t90, t100 = first(0.1 * target), first(0.9 * target), first(0.999 * target)
        overshoot = max(zi.max() - target, 0.0)
        outside = np.where(after & (np.abs(zi - target) > band))[0]
        settle = (t[outside[-1]] - t_step) if len(outside) else 0.0
        if len(outside) and outside[-1] >= len(t) - 1:
            settle = np.inf
        tail = t >= t[-1] - 2.0
        rms = np.sqrt(np.mean((zi[tail] - target) ** 2))
        pk = np.max(np.abs(zi[tail] - target))
        res.append(dict(t90=t90, t10_90=t90 - t10, overshoot=overshoot, settle=settle,
                        rms_tail=rms, peak_tail=pk))
    return res


# ---------------------------------------------------------------------------
# 4. REPORT
# ---------------------------------------------------------------------------
def physics_report():
    p = Nominal
    print("=" * 66)
    print("WEIGHT BUDGET")
    for name, m, src in BUDGET:
        print(f"  {name:<42s}{m:6.1f} g   [{src}]")
    tot = MASS_NOMINAL * 1000
    print(f"  {'TOTAL':<42s}{tot:6.1f} g")
    print(f"  with +15% contingency: {tot * 1.15:6.1f} g   (requirement: < 250 g)")
    print("=" * 66)
    print("THRUST, CURRENT, THROTTLE  (Ct is the main unknown)")
    print(f"  {'Ct':>5} {'Tmax/motor':>11} {'T/W':>6} {'Imax total':>11} {'hover duty':>11} "
          f"{'hover I':>8} {'hover time':>11}")
    for Ct in (0.07, 0.09, 0.12, 0.15):
        full = steady_state(1.0, 12.0, Ct)
        hd, hp = hover_point(Ct, MASS_NOMINAL)
        tw = 4 * full["T_N"] / (MASS_NOMINAL * G)
        I_h = hp["I_total"] + 0.25          # + electronics
        mins = 0.8 * 0.650 / I_h * 60       # 80% usable capacity
        print(f"  {Ct:5.2f} {full['T_g']:9.0f} g {tw:6.1f} {full['I_total']:9.1f} A "
              f"{hd * 100:9.0f} % {I_h:6.1f} A {mins:8.1f} min")
    print("  Battery limit: 75C x 0.65 Ah = 48.75 A continuous")
    print("  Motor limit (seller): 18 A / 200 W each")


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    physics_report()

    # theoretical fastest 0->1 m: full thrust up, then zero thrust (gravity brakes)
    full = steady_state(1.0, 12.0)
    a_up = 4 * full["T_N"] / MASS_NOMINAL - G
    a_dn = G
    v2 = 1.0 / (1 / (2 * a_up) + 1 / (2 * a_dn))
    t_min = np.sqrt(v2) / a_up + np.sqrt(v2) / a_dn
    print("=" * 66)
    print(f"PHYSICAL LIMIT: climb accel {a_up:.1f} m/s^2, fastest possible 0->1 m stop = {t_min:.2f} s"
          f" (ignores motor lag)")
    print("=" * 66)
    print("STEP TEST: 0 -> 1 m, nominal parameters, one run per sensor")
    print(f"  {'sensor':<27}{'t to 90%':>9}{'overshoot':>11}{'settle +-2cm':>14}{'rms (last 2 s)':>16}{'peak':>8}")
    runs = {}
    for name in SENSORS:
        o = simulate(name, N=1)
        runs[name] = o
        m = metrics(o)[0]
        st = "never" if np.isinf(m["settle"]) else f"{m['settle']:.2f} s"
        print(f"  {name:<27}{m['t90']:7.2f} s{m['overshoot'] * 100:8.1f} cm{st:>14}"
              f"{m['rms_tail'] * 1000:13.1f} mm{m['peak_tail'] * 1000:6.0f} mm")

    print("  (ESC passive coast-down assumed. With Bluejay active braking enabled:)")
    for name in ("Mocap-grade (ideal)", "Laser ToF (VL53-class)"):
        mb = metrics(simulate(name, N=1, brake=True))[0]
        print(f"  {name:<27}{mb['t90']:7.2f} s{mb['overshoot'] * 100:8.1f} cm{mb['settle']:11.2f} s"
              f"{mb['rms_tail'] * 1000:13.1f} mm{mb['peak_tail'] * 1000:6.0f} mm")
    print("=" * 66)
    print("ROBUSTNESS: 300 runs each, mass +-10%, Ct +-20%, Kv +-5%, battery 11.4-12.6 V, accel bias")
    mc = {}
    for name in SENSORS:
        o = simulate(name, N=300, spread=True, seed=7)
        mc[name] = metrics(o)
        ms = mc[name]
        ok = [m for m in ms if m["peak_tail"] <= 0.02 and np.isfinite(m["settle"])]
        st = np.array([m["settle"] for m in ms])
        st = st[np.isfinite(st)]
        med = np.median(st) if len(st) else float("nan")
        p95 = np.percentile([m["peak_tail"] for m in ms], 95) * 1000
        print(f"  {name:<27} within +-2 cm after settling: {100 * len(ok) / len(ms):5.1f}%   "
              f"median settle {med:4.2f} s   95th pct peak error {p95:6.1f} mm")

    # ------------------------- plot
    fig, ax = plt.subplots(2, 2, figsize=(13, 8.5))
    cols = {"Mocap-grade (ideal)": "#1f77b4", "Laser ToF (VL53-class)": "#2ca02c",
            "Barometer (BMP/DPS-class)": "#d62728"}
    for name, o in runs.items():
        ax[0, 0].plot(o["t"], o["z"][:, 0], color=cols[name], label=name)
        ax[0, 1].plot(o["t"], o["z"][:, 0], color=cols[name], label=name)
        ax[1, 0].plot(o["t"], o["duty"][:, 0] * 100, color=cols[name], lw=0.8)
    ax[0, 0].axhline(1.0, color="k", lw=0.5, ls=":")
    ax[0, 0].set(title="0 to 1 m step (command at 0.5 s)", xlabel="time (s)", ylabel="altitude (m)")
    ax[0, 0].legend(fontsize=8)
    ax[0, 1].axhspan(0.98, 1.02, color="gold", alpha=0.35, label="+-2 cm band")
    ax[0, 1].set(title="Zoom on the target band", xlabel="time (s)", ylabel="altitude (m)",
                 ylim=(0.9, 1.1), xlim=(0.5, 6))
    ax[1, 0].set(title="ESC duty (throttle) during the step", xlabel="time (s)", ylabel="duty (%)")
    for name, ms in mc.items():
        pk = np.array([m["peak_tail"] for m in ms]) * 1000
        ax[1, 1].hist(np.clip(pk, 0, 200), bins=40, alpha=0.55, color=cols[name], label=name)
    ax[1, 1].axvline(20, color="k", ls="--", lw=1)
    ax[1, 1].set(title="Worst hold error in last 2 s, 300 runs (dashed = 20 mm = +-2 cm)",
                 xlabel="peak error (mm), clipped at 200", ylabel="runs")
    ax[1, 1].legend(fontsize=8)
    for a in ax.flat:
        a.grid(alpha=0.25)
    plt.tight_layout()
    plt.savefig("wasp_results.png", dpi=130)
    print("saved wasp_results.png")
