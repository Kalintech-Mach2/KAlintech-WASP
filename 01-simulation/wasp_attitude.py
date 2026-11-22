"""
WASP Phase 1 - attitude (roll and pitch) model and tilt controller.

Idea in one paragraph
  The four motors sit at the corners of a square. If the left motors push harder
  than the right motors the drone rolls. The controller measures the tilt with a
  gyro (fast, drifts) and an accelerometer (slow, noisy), compares it with the
  tilt you asked for, and decides how much extra thrust each motor should give.

Run:  python3 wasp_attitude.py       (needs numpy, matplotlib)

Labels:  SELLER = from a datasheet, ASSUMED = my estimate, replace when measured.

Simplifications (written down so we remember to remove them later)
  - Altitude is NOT simulated here: total thrust is held at hover. Roll/pitch only.
  - No yaw (spin) yet.
  - Angles stay small (under about 20 degrees), so sin(x) is treated as x.
  - Accelerometer is assumed to see the tilt (see K_ACC). In a real flying drone
    it mostly cannot, because thrust always points along the drone's own "up".
    Test 4 shows what happens when you switch that help off.
"""
import numpy as np

RHO, G = 1.225, 9.81

# ----------------------------- airframe -----------------------------------
MASS = 0.1649                 # kg, from the weight budget
D = 0.0762                    # prop diameter, m
A = 0.060 / np.sqrt(2)        # motor offset from the centre in x and y (120 mm diagonal wheelbase)
IXX = IYY = 1.6e-4            # kg m^2, ASSUMED. Fusion 360 can give the real value later.
# motor positions, x forward, y left:   1 front-left, 2 front-right, 3 rear-left, 4 rear-right
POS_X = np.array([+1, +1, -1, -1]) * A
POS_Y = np.array([+1, -1, +1, -1]) * A

# ----------------------------- motor + prop (same numbers as wasp_sim) ----
KV, RM, I0 = 3750.0, 0.126, 0.6
CT, FOM, J, RB, VOC = 0.12, 0.44, 9e-7, 0.04, 12.0
KE = 60.0 / (2 * np.pi * KV)


def cq(ct):
    """torque coefficient from thrust coefficient (momentum theory, see wasp_sim)"""
    return 0.7979 * ct ** 1.5 / FOM / (2 * np.pi)


# ----------------------------- controller gains ---------------------------
GAINS = dict(
    Ka=8.0,        # angle loop: wanted rate (rad/s) per rad of angle error
    Kp=30.0,       # rate loop P: angular acceleration per rad/s of rate error
    Ki=60.0,       # rate loop I
    Kd=0.004,      # rate loop D (on the measurement)
    rate_max=8.0,  # rad/s, limit on wanted rate
)

# ----------------------------- sensor numbers (ASSUMED) -------------------
GYRO_NOISE = 0.02     # rad/s per sample, includes motor vibration
GYRO_BIAS = 0.01      # rad/s, about 0.6 deg/s
ACC_TILT_NOISE = 0.03  # rad, about 1.7 deg
K_ACC = 2.0           # 1/s, how strongly the accelerometer pulls the estimate. 0 = pure gyro.


def hover_duty():
    """duty each motor needs to hover, and the starting rotor speed"""
    T = MASS * G / 4
    n = np.sqrt(T / (CT * RHO * D ** 4))
    w = 2 * np.pi * n
    Q = cq(CT) * RHO * n ** 2 * D ** 5
    I = Q / KE + I0
    Vb = VOC - RB * 4 * I
    return (KE * w + I * RM) / Vb, w


def simulate(scale=(1, 1, 1, 1), dist=(0.0, 0.0), k_acc=K_ACC, gyro_bias=GYRO_BIAS,
             roll_cmd=lambda t: 0.0, pitch_cmd=lambda t: 0.0, T=6.0, seed=1, gains=GAINS):
    """
    scale : thrust multiplier per motor (1 = healthy, 0.85 = weak motor)
    dist  : constant disturbance torque (roll, pitch) in N m, e.g. a battery sitting off-centre
    """
    rng = np.random.default_rng(seed)
    dt, every = 5e-4, 2                     # physics step 0.5 ms, controller 1 kHz
    dtc = dt * every
    n_steps = int(T / dt)
    scale = np.array(scale, float)
    ct_i = CT * scale
    cq_i = cq(ct_i)

    duty_h, w_h = hover_duty()
    w = np.full(4, w_h)
    duty = np.full(4, duty_h)
    Vb = VOC
    phi = p = th = q = 0.0                  # roll angle/rate, pitch angle/rate
    phih = thh = 0.0                        # estimated angles
    Ir = Iq = 0.0                           # integrators
    pf = qf = 0.0                           # filtered gyro rates
    dp = dq = 0.0                           # filtered derivative of rate
    alpha_f = dtc / (dtc + 1 / (2 * np.pi * 80))      # 80 Hz low-pass on the gyro
    alpha_d = dtc / (dtc + 1 / (2 * np.pi * 40))      # 40 Hz low-pass on the D term

    names = ["t", "phi", "phih", "p", "th", "thh", "cmd", "duty", "thrust", "Lcmd"]
    log = {k: np.zeros((n_steps, 4 if k in ("duty", "thrust") else 1)) for k in names}
    Lcmd = Mcmd = 0.0
    for k in range(n_steps):
        t = k * dt
        # ---------------- physics: four motors ----------------
        n = w / (2 * np.pi)
        Q = cq_i * RHO * n ** 2 * D ** 5
        I = np.maximum((duty * Vb - KE * w) / RM, 0.0)
        Tm = ct_i * RHO * n ** 2 * D ** 4
        Vb = VOC - RB * I.sum()
        w = np.maximum(w + dt * (KE * (I - I0 * (w > 1)) - Q) / J, 0.0)
        # ---------------- physics: tilt ----------------
        L = np.sum(POS_Y * Tm)              # roll torque: left motors minus right motors
        M = -np.sum(POS_X * Tm)             # pitch torque
        p += dt * (L + dist[0]) / IXX
        q += dt * (M + dist[1]) / IYY
        phi += dt * p
        th += dt * q

        # ---------------- sensors + controller at 1 kHz ----------------
        if k % every == 0:
            p_m = p + gyro_bias + GYRO_NOISE * rng.standard_normal()
            q_m = q + gyro_bias + GYRO_NOISE * rng.standard_normal()
            acc_r = phi + ACC_TILT_NOISE * rng.standard_normal()
            acc_p = th + ACC_TILT_NOISE * rng.standard_normal()
            # complementary filter: integrate the gyro, nudge towards the accelerometer
            phih += p_m * dtc + k_acc * (acc_r - phih) * dtc
            thh += q_m * dtc + k_acc * (acc_p - thh) * dtc
            # filtered rate and its derivative
            pf_old, qf_old = pf, qf
            pf += alpha_f * (p_m - pf)
            qf += alpha_f * (q_m - qf)
            dp += alpha_d * ((pf - pf_old) / dtc - dp)
            dq += alpha_d * ((qf - qf_old) / dtc - dq)
            # outer loop: angle error -> wanted rate
            rmax = gains["rate_max"]
            p_ref = np.clip(gains["Ka"] * (roll_cmd(t) - phih), -rmax, rmax)
            q_ref = np.clip(gains["Ka"] * (pitch_cmd(t) - thh), -rmax, rmax)
            # inner loop: rate error -> angular acceleration -> torque
            er, eq_ = p_ref - pf, q_ref - qf
            Ir = np.clip(Ir + gains["Ki"] * er * dtc, -300, 300)
            Iq = np.clip(Iq + gains["Ki"] * eq_ * dtc, -300, 300)
            ar = gains["Kp"] * er + Ir - gains["Kd"] * dp
            aq = gains["Kp"] * eq_ + Iq - gains["Kd"] * dq
            Lcmd, Mcmd = IXX * ar, IYY * aq
            # mixer: split the torque request between the four motors
            T_cmd = MASS * G / 4 + (POS_Y * Lcmd - POS_X * Mcmd) / (4 * A ** 2)
            T_cmd = np.maximum(T_cmd, 0.0)
            # thrust -> duty (inverse of the nominal motor model)
            n_c = np.sqrt(T_cmd / (CT * RHO * D ** 4))
            w_c = 2 * np.pi * n_c
            Ic = cq(CT) * RHO * n_c ** 2 * D ** 5 / KE + I0
            duty = np.clip((KE * w_c + Ic * RM) / Vb, 0.0, 1.0)

        log["t"][k] = t; log["phi"][k] = phi; log["phih"][k] = phih; log["p"][k] = p
        log["th"][k] = th; log["thh"][k] = thh; log["cmd"][k] = roll_cmd(t)
        log["duty"][k] = duty; log["thrust"][k] = Tm / G * 1000; log["Lcmd"][k] = Lcmd
    return {k: v.squeeze() if v.shape[1] == 1 else v for k, v in log.items()}


def step_metrics(o, t0=0.5, target=np.deg2rad(10.0), band=np.deg2rad(1.0)):
    t, phi = o["t"], o["phi"]
    after = t >= t0
    def first(level):
        i = np.where(after & (phi >= level))[0]
        return t[i[0]] - t0 if len(i) else np.inf
    rise = first(0.9 * target) - first(0.1 * target)
    # settle: judge on the first step only (0.5 s to 2.0 s)
    win = (t >= t0) & (t < 2.0)
    out = np.where(win & (np.abs(phi - target) > band))[0]
    settle = (t[out[-1]] - t0) if len(out) else 0.0
    over = max(phi[win].max() - target, 0.0)
    return dict(rise=rise, settle=settle, over_deg=np.rad2deg(over))


def sat_fraction(o):
    d = o["duty"]
    return float(np.mean(np.any((d >= 0.999) | (d <= 0.001), axis=1)))


TESTS = {
    "1 Clean roll step": dict(),
    "2 Battery 5 mm off-centre": dict(dist=(MASS * G * 0.005, 0.0)),
    "3 Weak motor (rear-right at 85%)": dict(scale=(1, 1, 1, 0.85)),
    "4 Gyro drift, accelerometer help off": dict(k_acc=0.0),
}
STEP = lambda t: np.deg2rad(10.0) if 0.5 <= t < 2.0 else 0.0

if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runs = {}
    print(f"{'test':<40}{'rise 10-90':>11}{'overshoot':>11}{'settle +-1deg':>14}{'angle error @5-6 s':>20}{'motors saturated':>18}")
    for name, kw in TESTS.items():
        o = simulate(roll_cmd=STEP, **kw)
        runs[name] = o
        m = step_metrics(o)
        tail = o["t"] >= 5.0
        err = np.rad2deg(np.mean(np.abs(o["phi"][tail])))
        print(f"{name:<40}{m['rise'] * 1000:8.0f} ms{m['over_deg']:8.2f} deg{m['settle'] * 1000:11.0f} ms"
              f"{err:15.2f} deg{100 * sat_fraction(o):13.1f} %")

    for i, (name, o) in enumerate(runs.items()):
        fig, ax = plt.subplots(2, 2, figsize=(12.5, 8))
        t = o["t"]
        ax[0, 0].plot(t, np.rad2deg(o["cmd"]), "k--", label="command")
        ax[0, 0].plot(t, np.rad2deg(o["phi"]), label="true roll")
        ax[0, 0].plot(t, np.rad2deg(o["phih"]), alpha=0.6, label="estimated roll")
        ax[0, 0].set(title="Roll angle", xlabel="time (s)", ylabel="degrees"); ax[0, 0].legend()
        ax[0, 1].plot(t, np.rad2deg(o["p"]))
        ax[0, 1].set(title="Roll rate", xlabel="time (s)", ylabel="degrees per second")
        for j, lab in enumerate(["front-left", "front-right", "rear-left", "rear-right"]):
            ax[1, 0].plot(t, o["duty"][:, j] * 100, lw=0.9, label=lab)
        ax[1, 0].axhline(100, color="r", ls=":"); ax[1, 0].axhline(0, color="r", ls=":")
        ax[1, 0].set(title="Motor throttle (red dotted = limits)", xlabel="time (s)", ylabel="duty %")
        ax[1, 0].legend(fontsize=8)
        ax[1, 1].plot(t, np.rad2deg(o["th"]), label="pitch angle")
        ax[1, 1].plot(t, np.rad2deg(o["thh"]), alpha=0.6, label="estimated pitch")
        ax[1, 1].set(title="Pitch angle (should stay near 0)", xlabel="time (s)", ylabel="degrees")
        ax[1, 1].legend()
        for a in ax.flat: a.grid(alpha=0.25)
        fig.suptitle("Test " + name)
        plt.tight_layout()
        plt.savefig(f"attitude_test{name[0]}.png", dpi=110)
        plt.close(fig)
    print("saved attitude_test1..4.png")
