"""
WASP micro (1S brushed quad) - indoor flight simulation, no wind.

Python stand-in for a Simulink model. Same block structure:
  pilot/test script -> attitude + altitude controller -> mixer -> duty cycles
  -> brushed-motor + prop model (with battery sag) -> rigid body -> sensors
  -> estimator -> back to the controller.

Run:  python3 wasp_micro_sim.py            (numpy, scipy, matplotlib)

Labels: LISTING = from a seller page, BUDGET = wasp_budget.py,
        ASSUMED = my estimate. Replace ASSUMED with measurements when parts arrive.
The numbers that matter most and are NOT known yet:
  TMAX_G (thrust per motor), motor resistance, battery internal resistance,
  prop torque ratio CQ, real inertia (take it from Fusion 360).
Everything here is conditional on those. The sweeps show how much each one matters.
"""
import numpy as np

G = 9.81
DT = 0.002                    # 500 Hz physics and control loop

# ------------------------------------------------------------------ mass budget
# (name, grams, box size mm (x, y, z), position mm (x, y, z))   x forward, y left, z up
def components(pcb_g):
    c = [("motor+prop FL", 5.5, None, (+35, +35, 0)),
         ("motor+prop FR", 5.5, None, (+35, -35, 0)),
         ("motor+prop RL", 5.5, None, (-35, +35, 0)),
         ("motor+prop RR", 5.5, None, (-35, -35, 0)),
         ("battery 1S 450", 11.0, (62, 17, 6), (0, 0, -6)),
         ("ESP32 SuperMini", 2.5, (22, 18, 4), (-8, 0, 5)),
         ("MPU-6050", 2.0, (20, 16, 3), (8, 0, 4)),
         ("VL53L1X", 1.5, (12, 10, 3), (0, 0, -9)),
         ("frame arm FL", 2.0, None, (17, 17, 0)),
         ("frame arm FR", 2.0, None, (17, -17, 0)),
         ("frame arm RL", 2.0, None, (-17, 17, 0)),
         ("frame arm RR", 2.0, None, (-17, -17, 0)),
         ("mosfets, caps, wire, lead, screws",
          0.6 + 1.0 + 1.0 + 0.5 + 1.5, (30, 20, 3), (0, 0, 2))]
    if pcb_g > 0:
        c.append(("custom PCB", pcb_g, (40, 30, 1), (0, 0, 2)))
    return c


def mass_inertia(pcb_g=5.0):
    """total mass kg, diagonal inertia kg m^2 about the centre of mass"""
    comps = components(pcb_g)
    m = sum(c[1] for c in comps) / 1000
    cg = sum(np.array(c[3]) * c[1] for c in comps) / (m * 1000)  # mm
    I = np.zeros(3)
    for _, g, box, pos in comps:
        mm = g / 1000
        r = (np.array(pos) - cg) / 1000
        if box is not None:
            dx, dy, dz = np.array(box) / 1000
            I += mm * np.array([dy**2 + dz**2, dx**2 + dz**2, dx**2 + dy**2]) / 12
        I += mm * np.array([r[1]**2 + r[2]**2, r[0]**2 + r[2]**2, r[0]**2 + r[1]**2])
    return m, I


def pcb_mass_g(w_mm=30, l_mm=40, t_mm=1.0, copper_cover=0.5, layers=2, extras_g=0.3):
    """bare FR4 + copper + headers/solder. FR4 1.85 g/cm3, copper 8.96 g/cm3, 35 um"""
    area = w_mm * l_mm / 100.0                      # cm2
    fr4 = area * t_mm / 10 * 1.85
    cu = area * 0.0035 * 8.96 * copper_cover * layers
    return fr4 + cu + extras_g


# ------------------------------------------------------------------ nominal parameters
KE = 3.7 / (50000 * 2 * np.pi / 60)       # V s/rad, from 3.7 V -> 50,000 rpm no-load (LISTING)
NOM = dict(
    tmax_g=25.0,        # thrust per motor, full throttle, fresh pack, ASSUMED  (MEASURE)
    v_cal=4.15,         # volts at the motor when that thrust was measured (one motor on a fresh pack)
    cq=0.0055,          # prop torque per newton of thrust (m), ASSUMED. gives ~4 g/W at hover
    Rm=1.2,             # motor resistance ohm, ASSUMED
    ke=KE,              # V s/rad, LISTING
    i0=0.12,            # no-load current A, ASSUMED
    Jr=5e-8,            # rotor + prop inertia kg m^2, ASSUMED (gives ~45 ms spin-up)
    Rb=0.10,            # battery + connector + wire resistance ohm, ASSUMED
    cap_ah=0.45 * 0.97, # pack capacity
    soc=1.0,
    a=0.035,            # motor offset from centre in x and y (70 mm adjacent spacing), BUDGET
    cd=0.03,            # horizontal drag kg/s (body + rotor drag), ASSUMED
    cz=0.02,            # vertical drag kg/s, ASSUMED
    rot_damp=2e-5,      # N m s/rad
    gyro_noise=0.03,    # rad/s per sample incl. vibration, ASSUMED
    gyro_bias=0.002,    # rad/s residual after the boot calibration (about 0.11 deg/s), ASSUMED
    acc_noise=0.5,      # m/s2 per sample incl. vibration, ASSUMED
    n_delay=2,          # control-to-motor latency in 2 ms steps (4 ms)
    k_acc=0.2,          # accelerometer pull on the attitude estimate (1/s)
    dkt=np.zeros(4),    # per-motor thrust spread (fraction)
    cg_dx=0.0, cg_dy=0.0,   # centre-of-gravity offset m
    mass_err=0.0,       # true mass vs. mass the controller thinks
)
VOC_SOC = [0.0, 0.05, 0.15, 0.30, 0.50, 0.70, 0.90, 1.0]
VOC_V = [3.30, 3.55, 3.70, 3.80, 3.92, 4.05, 4.20, 4.35]   # 1S LiHV open-circuit, ASSUMED


def cal(tmax_g, cq, Rm, ke, i0, v_cal):
    """prop thrust coefficient so that full throttle at v_cal gives tmax_g.
    returns kT (N/(rad/s)^2), kQ, tau_friction"""
    kt = ke
    tf = kt * i0
    T = tmax_g / 1000 * G
    i = (cq * T + tf) / kt
    w = (v_cal - Rm * i) / ke
    kT = T / w**2
    return kT, cq * kT, tf


def full_thrust(V, kT, kQ, tf, Rm, ke):
    """steady thrust at 100 % duty and battery voltage V (closed form)"""
    kt = ke
    a = Rm * kQ / kt
    c = Rm * tf / kt - V
    w = (-ke + np.sqrt(ke**2 - 4 * a * c)) / (2 * a)
    return kT * w**2


def duty_for(T, V, kT, kQ, tf, Rm, ke):
    """steady duty to get thrust T (N) at battery V"""
    kt = ke
    w = np.sqrt(np.maximum(T, 0) / kT)
    i = (kQ * w**2 + tf) / kt
    return (Rm * i + ke * w) / V


# ------------------------------------------------------------------ small batched maths
def qmul(a, b):
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack([aw*bw - ax*bx - ay*by - az*bz,
                     aw*bx + ax*bw + ay*bz - az*by,
                     aw*by - ax*bz + ay*bw + az*bx,
                     aw*bz + ax*by - ay*bx + az*bw], axis=1)


def q2R(q):
    w, x, y, z = q.T
    R = np.empty((len(w), 3, 3))
    R[:, 0, 0] = 1 - 2*(y*y + z*z); R[:, 0, 1] = 2*(x*y - w*z); R[:, 0, 2] = 2*(x*z + w*y)
    R[:, 1, 0] = 2*(x*y + w*z); R[:, 1, 1] = 1 - 2*(x*x + z*z); R[:, 1, 2] = 2*(y*z - w*x)
    R[:, 2, 0] = 2*(x*z - w*y); R[:, 2, 1] = 2*(y*z + w*x); R[:, 2, 2] = 1 - 2*(x*x + y*y)
    return R


def euler(q):
    w, x, y, z = q.T
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


def q_from_euler(roll, pitch, yaw):
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    return np.stack([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                     cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy], axis=1)


# ------------------------------------------------------------------ parameter sets
def make_params(N=1, pcb_g=5.0, mass_extra_g=0.0, **over):
    """N parameter sets, each entry an array of length N (or (N,4) / (N,3)).
    over: scalar or length-N array for any key in NOM."""
    base = dict(NOM)
    base.update(over)
    m0, I0 = mass_inertia(pcb_g)
    extra = np.broadcast_to(np.asarray(mass_extra_g, float) / 1000, (N,)).copy()
    p = {"N": N}
    p["mass"] = m0 + extra
    p["I"] = np.outer(p["mass"] / m0, I0)
    for k, v in base.items():
        v = np.asarray(v, dtype=float)
        if k == "dkt":
            p[k] = np.tile(v, (N, 1)) if v.ndim == 1 else v
        else:
            p[k] = np.full(N, float(v)) if v.ndim == 0 else v
    return p


# default controller gains, tuned in tune_gains() on the nominal vehicle
GAINS = dict(Ka=4.9, Kp=27.7, Ki=51.0, Kd=0.0069, Kyaw=10.0, Kpa=2.0,
             rate_max=8.0, Kz=0.55, Kvz=0.20, Kiz=0.25, tilt_lim=np.radians(45))


# ------------------------------------------------------------------ the simulation
def simulate(P, scen, T_end, gains=None, dec=10, mode="alt", init=None, tof=True):
    """
    P     parameter dict from make_params
    scen  function(t) -> dict with arrays: roll, pitch, yawrate, zc, thr (thrust adjust fraction),
          and optional kick (N,3) body-rate impulse, applied once at that step
    mode  'alt' = ToF altitude hold, 'manual' = pilot holds hover-ish thrust, no altitude loop
    init  dict: z, roll, pitch, w (N,3), on_ground
    """
    g = dict(GAINS)
    if gains:
        g.update(gains)
    N = P["N"]
    mass_c = P["mass"] / (1 + P["mass_err"])   # what the controller believes
    mass = P["mass"]
    Itrue = P["I"]
    Inom = Itrue.copy()                         # controller uses design inertia

    # true motor constants per motor
    kT0, kQ0, tf = cal(P["tmax_g"], P["cq"], P["Rm"], P["ke"], P["i0"], P["v_cal"])
    kT = kT0[:, None] * (1 + P["dkt"])
    kQ = P["cq"][:, None] * kT
    Rm, ke, Jr = P["Rm"], P["ke"], P["Jr"]
    # controller's nominal map (no per-motor spread)
    kTn, kQn, tfn = kT0, kQ0, tf

    a = P["a"]
    xm = np.array([+1, +1, -1, -1])[None, :] * a[:, None]          # FL FR RL RR, shape (N,4)
    ym = np.array([+1, -1, +1, -1])[None, :] * a[:, None]
    sg = np.array([+1, -1, -1, +1])              # spin direction, + = counter-clockwise
    xm_t = xm - P["cg_dx"][:, None]
    ym_t = ym - P["cg_dy"][:, None]

    # state
    init = init or {}
    z = np.full(N, init.get("z", 1.0))
    pos = np.zeros((N, 3)); pos[:, 2] = z
    vel = np.zeros((N, 3))
    q = q_from_euler(np.broadcast_to(init.get("roll", 0.0), (N,)).astype(float),
                     np.broadcast_to(init.get("pitch", 0.0), (N,)).astype(float), np.zeros(N))
    w = np.zeros((N, 3)) + init.get("w", 0.0)
    q_est = q_from_euler(np.zeros(N), np.zeros(N), np.zeros(N)) if init.get("blind") else q.copy()
    soc = P["soc"].copy()
    on_ground = init.get("on_ground", False)

    Voc = lambda s: np.interp(s, VOC_SOC, VOC_V)
    V = Voc(soc) - P["Rb"] * 3.0
    Thov = mass * G / 4
    om = np.zeros((N, 4)) if on_ground else np.sqrt(Thov[:, None] / kT)

    rng = np.random.default_rng(1)
    gyro_bias = rng.normal(0, P["gyro_bias"][:, None], (N, 3))
    gyro_f = np.zeros((N, 3)); gyro_prev = np.zeros((N, 3)); dgyro = np.zeros((N, 3))
    acc_f = np.zeros((N, 3)); acc_f[:, 2] = G
    Iint = np.zeros((N, 3)); zint = np.zeros(N)
    z_est = z.copy(); vz_est = np.zeros(N); z_hist = []
    z_meas_last = z.copy()
    delay = P["n_delay"].astype(int)
    nbuf = int(delay.max()) + 2
    dbuf = np.zeros((nbuf, N, 4))
    duty_cmd = np.zeros((N, 4)) if on_ground else duty_for(Thov[:, None], V[:, None], kTn[:, None], kQn[:, None], tfn[:, None], Rm[:, None], ke[:, None])
    dbuf[:] = duty_cmd
    tof_buf = [z.copy()] * 16
    I_bat = np.full(N, 3.0)
    a_ng = np.zeros((N, 3))
    az_f = np.zeros(N)

    tof_n = max(1, int(round(0.03 / DT)))
    alpha_g = 1 - np.exp(-DT / 0.0025)
    alpha_a = 1 - np.exp(-DT / 0.008)
    alpha_d = 1 - np.exp(-DT / 0.004)
    alpha_V = 1 - np.exp(-DT / 0.05)         # ADC voltage reading is filtered
    Vm = V.copy()

    steps = int(round(T_end / DT))
    if on_ground:
        pos[:, 2] = 0.0
    nrec = steps // dec + 1
    rec = {k: np.zeros((nrec, N)) for k in
           ["t", "x", "y", "z", "tilt", "tilt_est", "roll", "pitch", "yaw", "vz", "duty_max", "duty_min",
            "V", "I", "soc", "sat", "T_av", "ztof"]}
    rec["wr"] = np.zeros((nrec, N, 3))
    ri = 0
    impacted = np.zeros(N, bool); vimpact = np.zeros(N)
    flipped = np.zeros(N, bool)
    airborne = np.zeros(N, bool) | (not on_ground)
    sat_flag = np.zeros(N)

    for k in range(steps + 1):
        t = k * DT
        cmd = scen(t)
        if "kick" in cmd and cmd["kick"] is not None:
            w = w + cmd["kick"]

        # ---------------- sensors (read the state of the previous step)
        R = q2R(q)
        gyro_m = w + gyro_bias + rng.normal(0, 1, (N, 3)) * P["gyro_noise"][:, None]
        gyro_f += alpha_g * (gyro_m - gyro_f)
        dg = (gyro_f - gyro_prev) / DT
        gyro_prev = gyro_f.copy()
        dgyro += alpha_d * (dg - dgyro)
        f_body = np.einsum("nji,nj->ni", R, a_ng)                 # specific force in body frame
        acc_m = f_body + rng.normal(0, 1, (N, 3)) * P["acc_noise"][:, None]
        acc_f += alpha_a * (acc_m - acc_f)

        # ---------------- attitude estimator (complementary filter)
        Re = q2R(q_est)
        u_est = Re[:, 2, :]                                       # world up seen from body, estimate
        n_acc = np.linalg.norm(acc_f, axis=1)
        u_meas = acc_f / np.maximum(n_acc, 1e-6)[:, None]
        ok = (n_acc > 2.0)[:, None]
        w_corr = np.where(ok, P["k_acc"][:, None] * np.cross(u_meas, u_est), 0.0)
        wt = gyro_f - gyro_bias * 0 + w_corr
        dq = np.concatenate([np.zeros((N, 1)), wt], axis=1)
        q_est = q_est + 0.5 * DT * qmul(q_est, dq)
        q_est /= np.linalg.norm(q_est, axis=1)[:, None]
        r_e, p_e, y_e = euler(q_est)

        # ---------------- ToF altitude sensor: 33 Hz, 30 ms latency, 4 mm noise
        if k % tof_n == 0:
            Rt = R[:, 2, 2]
            slant = np.maximum(pos[:, 2], 0) / np.maximum(Rt, 0.3)
            tof_buf.append(slant + rng.normal(0, 0.004, N))
            tof_buf.pop(0)
            zm = tof_buf[-2] * np.cos(r_e) * np.cos(p_e)          # tilt-compensated from estimate
            innov = zm - z_est
            z_est += 0.5 * innov
            vz_est += 0.25 * innov / 0.03
        z_est += vz_est * DT
        if not tof:
            z_est = pos[:, 2].copy(); vz_est = vel[:, 2].copy()

        # ---------------- controller
        roll_c = np.clip(cmd["roll"], -g["tilt_lim"], g["tilt_lim"])
        pitch_c = np.clip(cmd["pitch"], -g["tilt_lim"], g["tilt_lim"])
        p_c = np.clip(g["Ka"] * (roll_c - r_e), -g["rate_max"], g["rate_max"])
        q_c = np.clip(g["Ka"] * (pitch_c - p_e), -g["rate_max"], g["rate_max"])
        r_c = cmd["yawrate"] + g["Kpa"] * (0 - y_e) * 0
        wc = np.stack([p_c, q_c, r_c], axis=1)
        e = wc - gyro_f
        Iint += g["Ki"] * e * DT * (sat_flag < 0.5)[:, None]
        Iint = np.clip(Iint, -60, 60)
        Kp = np.array([g["Kp"], g["Kp"], g["Kyaw"]])
        Kd = np.array([g["Kd"], g["Kd"], 0.0])
        alpha_cmd = Kp * e + Iint - Kd * dgyro
        tau_c = Inom * alpha_cmd

        cosT = np.maximum(np.cos(r_e) * np.cos(p_e), 0.5)
        if mode == "alt":
            ez = cmd["zc"] - z_est
            zint = np.clip(zint + g["Kiz"] * ez * DT, -0.3 * mass_c * G, 0.3 * mass_c * G)
            Tc = (mass_c * G + g["Kz"] * mass_c / 0.0566 * ez - g["Kvz"] * mass_c / 0.0566 * vz_est + zint) / cosT
        else:
            Tc = mass_c * G * (1 + cmd["thr"]) / cosT
        Tc = np.maximum(Tc, 0)

        # mixer: per-motor thrust demands
        Ti_att = (ym * tau_c[:, 0:1] - xm * tau_c[:, 1:2]) / (4 * a[:, None]**2) \
                 - sg[None, :] * tau_c[:, 2:3] / (4 * P["cq"][:, None])
        Vm += alpha_V * (V - Vm)
        Tav = full_thrust(np.maximum(Vm, 2.5), kTn, kQn, tfn, Rm, ke)   # thrust at 100 % duty now
        Tmin = 0.0006 * G
        lo = Tmin - Ti_att.min(axis=1)
        hi = Tav - Ti_att.max(axis=1)
        spread = Ti_att.max(axis=1) - Ti_att.min(axis=1)
        room = Tav - Tmin
        fits = lo <= hi
        scale = np.where(fits, 1.0, room / np.maximum(spread, 1e-9))
        Ti_att = Ti_att * scale[:, None]
        lo = Tmin - Ti_att.min(axis=1); hi = Tav - Ti_att.max(axis=1)
        coll = np.clip(Tc / 4, lo, np.maximum(hi, lo))
        Ti = coll[:, None] + Ti_att
        sat_flag = (~fits) | (Tc / 4 > hi) | (Tc / 4 < lo)
        Ti = np.clip(Ti, Tmin, Tav[:, None])
        duty_c = duty_for(Ti, np.maximum(Vm, 2.5)[:, None], kTn[:, None], kQn[:, None], tfn[:, None],
                          Rm[:, None], ke[:, None])
        duty_c = np.clip(duty_c, 0.0, 1.0)
        if on_ground and t < cmd.get("spool_t", 0.0):
            duty_c *= 0
        dbuf = np.roll(dbuf, 1, axis=0); dbuf[0] = duty_c
        duty = dbuf[np.minimum(delay, nbuf - 1), np.arange(N)]

        # ---------------- motors and battery
        i_m = np.maximum(0.0, (duty * V[:, None] - ke[:, None] * om) / Rm[:, None])
        Qe = ke[:, None] * i_m
        Tq = Qe - kQ * om**2 - np.where(om > 0, tf[:, None], 0.0)
        dom = Tq / Jr[:, None]
        stuck = (om <= 0) & (dom < 0)
        om = np.maximum(om + DT * np.where(stuck, 0, dom), 0)
        I_bat = (duty * i_m).sum(axis=1) + 0.11
        soc = soc - I_bat * DT / 3600 / P["cap_ah"]
        V = Voc(np.clip(soc, 0, 1)) - P["Rb"] * I_bat
        V = np.maximum(V, 0.5)

        # ---------------- rigid body
        Tm = kT * om**2
        Ttot = Tm.sum(axis=1)
        Fb = np.zeros((N, 3)); Fb[:, 2] = Ttot
        Fw = np.einsum("nij,nj->ni", R, Fb)
        drag = np.stack([-P["cd"] * vel[:, 0], -P["cd"] * vel[:, 1], -P["cz"] * vel[:, 2]], axis=1)
        a_ng = (Fw + drag) / mass[:, None]
        acc = a_ng.copy(); acc[:, 2] -= G
        tau = np.stack([(ym_t * Tm).sum(axis=1), -(xm_t * Tm).sum(axis=1),
                        -(sg[None, :] * (kQ * om**2 + Jr[:, None] * dom)).sum(axis=1)], axis=1)
        tau -= P["rot_damp"][:, None] * w
        Iw = Itrue * w
        w = w + DT * (tau - np.cross(w, Iw)) / Itrue
        dqq = np.concatenate([np.zeros((N, 1)), w], axis=1)
        q = q + 0.5 * DT * qmul(q, dqq)
        q /= np.linalg.norm(q, axis=1)[:, None]
        vel = vel + DT * acc
        pos = pos + DT * vel

        # floor
        low = pos[:, 2] <= 0
        if low.any():
            if airborne.any():
                hit = low & airborne & (vel[:, 2] < -0.8) & (~impacted)
                vimpact = np.where(hit, vel[:, 2], vimpact)
                impacted |= hit
            pos[low, 2] = 0
            vel[low, 2] = np.maximum(vel[low, 2], 0)
            vel[low, 0:2] *= 0.9
            w[low] *= 0.9
        airborne |= pos[:, 2] > 0.05
        tilt = np.degrees(np.arccos(np.clip(R[:, 2, 2], -1, 1)))
        flipped |= (tilt > 75) & airborne & (pos[:, 2] > 0.05)

        if k % dec == 0:
            rr, pp, yy = euler(q)
            rec["t"][ri] = t; rec["x"][ri] = pos[:, 0]; rec["y"][ri] = pos[:, 1]; rec["z"][ri] = pos[:, 2]
            rec["tilt"][ri] = tilt
            tilt_e = np.degrees(np.arccos(np.clip(q2R(q_est)[:, 2, 2], -1, 1)))
            rec["tilt_est"][ri] = tilt_e
            rec["roll"][ri] = np.degrees(rr); rec["pitch"][ri] = np.degrees(pp); rec["yaw"][ri] = np.degrees(yy)
            rec["vz"][ri] = vel[:, 2]
            rec["duty_max"][ri] = duty.max(axis=1); rec["duty_min"][ri] = duty.min(axis=1)
            rec["V"][ri] = V; rec["I"][ri] = I_bat; rec["soc"][ri] = soc
            rec["sat"][ri] = sat_flag.astype(float); rec["T_av"][ri] = Tav * 4 / (mass * G)
            rec["ztof"][ri] = z_est
            rec["wr"][ri] = w
            ri += 1
    for kk in rec:
        rec[kk] = rec[kk][:ri]
    rec["flipped"] = flipped; rec["impacted"] = impacted; rec["vimpact"] = vimpact
    return rec


# ------------------------------------------------------------------ pilot scripts
def const(roll=0.0, pitch=0.0, yawrate=0.0, zc=1.0, thr=0.0, N=1):
    def f(t):
        return dict(roll=np.full(N, roll), pitch=np.full(N, pitch), yawrate=np.full(N, yawrate),
                    zc=np.full(N, zc), thr=np.full(N, thr))
    return f


def piecewise(N, segs, zc=1.0, kicks=None):
    """segs: list of (t_start, roll_deg, pitch_deg) ; kicks: list of (t, (N,3) array rad/s)"""
    kicks = kicks or []
    done = set()

    def f(t):
        r = p = 0.0
        for ts, rd, pd in segs:
            if t >= ts:
                r, p = np.radians(rd), np.radians(pd)
        out = dict(roll=np.broadcast_to(r, (N,)).astype(float) if np.ndim(r) == 0 else r,
                   pitch=np.broadcast_to(p, (N,)).astype(float) if np.ndim(p) == 0 else p,
                   yawrate=np.zeros(N), zc=np.full(N, zc), thr=np.zeros(N), kick=None)
        for i, (tk, kv) in enumerate(kicks):
            if t >= tk and i not in done:
                out["kick"] = kv
                done.add(i)
        return out
    return f
