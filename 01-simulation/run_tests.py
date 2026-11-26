"""
WASP micro indoor test suite (no wind). Run: python3 run_tests.py
Writes results.json, wasp_sim_fig1.png, wasp_sim_fig2.png and prints a summary.
Every number depends on the ASSUMED motor / battery values in wasp_micro_sim.py.
"""
import json, sys, time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import wasp_micro_sim as W

G = W.G
out = {}
plt.rcParams.update({"font.size": 8, "axes.grid": True, "grid.alpha": 0.3})


def scen_leans(N, roll_deg, pitch_deg, t_on, t_off, zc=1.0):
    roll_deg = np.broadcast_to(np.asarray(roll_deg, float), (N,))
    pitch_deg = np.broadcast_to(np.asarray(pitch_deg, float), (N,))

    def f(t):
        on = t_on <= t < t_off
        return dict(roll=np.radians(roll_deg) * on, pitch=np.radians(pitch_deg) * on,
                    yawrate=np.zeros(N), zc=np.full(N, zc), thr=np.zeros(N))
    return f


# ======================================================================= T0  static numbers
print("T0 static numbers")
m_g = {}
for pcb in (0.0, 3.0, 5.0):
    m, _ = W.mass_inertia(pcb)
    m_g[pcb] = m * 1000
out["mass_g"] = {f"pcb_{k:.0f}g": round(v, 1) for k, v in m_g.items()}
out["mass_g"]["pcb_5g_plus_15pct"] = round(m_g[5.0] * 1.15, 1)
out["pcb_g"] = {"30x40x1.0mm": round(W.pcb_mass_g(), 2), "30x40x1.6mm": round(W.pcb_mass_g(t_mm=1.6), 2),
                "25x35x0.8mm": round(W.pcb_mass_g(25, 35, 0.8), 2), "40x40x1.6mm": round(W.pcb_mass_g(40, 40, 1.6), 2)}
P = W.make_params(1)
kT, kQ, tf = W.cal(P["tmax_g"], P["cq"], P["Rm"], P["ke"], P["i0"], P["v_cal"])
mass = P["mass"][0]
rows = []
for V in (4.15, 3.9, 3.7, 3.5, 3.3):
    Tf = float(W.full_thrust(V, kT, kQ, tf, P["Rm"], P["ke"])[0])
    twr = 4 * Tf / (mass * G)
    d_h = float(W.duty_for(mass * G / 4, V, kT, kQ, tf, P["Rm"], P["ke"])[0])
    # largest lean that keeps 15 % collective headroom for the attitude loop (needs W/cos <= 0.85 * available)
    c = (mass * G) / (0.85 * 4 * Tf)
    lean = float(np.degrees(np.arccos(c))) if c < 1 else 0.0
    rows.append(dict(V=V, thrust_per_motor_g=round(Tf / G * 1000, 1), twr=round(twr, 2), hover_duty=round(d_h, 2),
                     max_lean_deg_15pct_headroom=round(lean, 0)))
out["static_by_voltage"] = rows
for r in rows:
    print("  ", r)

# ======================================================================= T1  hover + drift
print("T1 hover (30 s) and drift distribution")
P = W.make_params(1)
r1 = W.simulate(P, W.const(), 30.0, dec=10)
out["hover"] = dict(max_tilt_deg=float(r1["tilt"].max()), hover_duty=float(r1["duty_max"][50:].mean()),
                    battery_current_A=float(r1["I"][50:].mean()), battery_V=float(r1["V"][50:].mean()),
                    z_err_mm=float(np.abs(r1["z"][500:] - 1.0).max() * 1000),
                    drift_30s_m=float(np.hypot(r1["x"][-1, 0], r1["y"][-1, 0])))
rng = np.random.default_rng(7)
Nd = 120
Pd = W.make_params(Nd, gyro_bias=np.abs(rng.normal(0, 0.003, Nd)), cg_dx=rng.normal(0, 0.0015, Nd),
                   cg_dy=rng.normal(0, 0.0015, Nd), dkt=np.clip(rng.normal(0, 0.07, (Nd, 4)), -0.15, 0.15),
                   mass_err=rng.uniform(-0.08, 0.08, Nd))
rd = W.simulate(Pd, W.const(N=Nd), 20.0, dec=50)
dist = np.hypot(rd["x"], rd["y"])
out["hover_drift_20s_m"] = dict(median=float(np.median(dist[-1])), p90=float(np.percentile(dist[-1], 90)),
                                max=float(dist[-1].max()), max_tilt_deg_p95=float(np.percentile(rd["tilt"].max(axis=0), 95)))
print("  ", out["hover"]); print("  ", out["hover_drift_20s_m"])

# ======================================================================= T2  takeoff + altitude hold
print("T2 takeoff from the floor, ToF altitude hold to 1.0 m")


def scen_takeoff(t):
    return dict(roll=np.zeros(1), pitch=np.zeros(1), yawrate=np.zeros(1),
                zc=np.array([np.clip((t - 0.5) * 1.0, 0, 1.0)]), thr=np.zeros(1))


r2 = W.simulate(P, scen_takeoff, 8.0, init=dict(z=0.0, on_ground=True), dec=10)
z2 = r2["z"][:, 0]; t2 = r2["t"][:, 0]
out["takeoff"] = dict(overshoot_cm=float(max(z2.max() - 1.0, 0) * 100), t_to_0p9m=float(t2[np.argmax(z2 > 0.9)]),
                      max_tilt_deg=float(r2["tilt"].max()), duty_max=float(r2["duty_max"].max()),
                      settle_error_cm=float(abs(z2[-1] - 1.0) * 100))
print("  ", out["takeoff"])

# ======================================================================= T3  leans (move without a dangerous tilt)
print("T3 lean / forward flight")
angs = np.array([10.0, 15.0, 20.0, 30.0, 40.0])
Nl = len(angs)
Pl = W.make_params(Nl)
rl = W.simulate(Pl, scen_leans(Nl, 0, angs, 1.0, 4.0), 7.0, dec=10)
lean_rows = []
for j, a in enumerate(angs):
    v = np.hypot(rl["x"][:, j], rl["y"][:, j])
    zmin = rl["z"][:, j].min()
    lean_rows.append(dict(commanded_deg=a, peak_tilt_deg=round(float(rl["tilt"][:, j].max()), 1),
                          altitude_dip_cm=round(float((1.0 - zmin) * 100), 1),
                          duty_peak=round(float(rl["duty_max"][:, j].max()), 2),
                          saturated_pct=round(float(rl["sat"][:, j].mean() * 100), 1),
                          distance_m=round(float(v[-1]), 1),
                          flipped=bool(rl["flipped"][j]), floor_hit=bool(rl["impacted"][j])))
out["lean_pitch_3s"] = lean_rows
for r in lean_rows:
    print("  ", r)
vxy = np.hypot(np.diff(rl["x"], axis=0), np.diff(rl["y"], axis=0)) / (10 * W.DT)
out["top_speed_mps_by_lean"] = {str(int(a)): round(float(vxy[:, j].max()), 1) for j, a in enumerate(angs)}

# yaw authority vs prop torque ratio
cqs = np.array([0.0045, 0.0055, 0.0075])
Py = W.make_params(3, cq=cqs)


def scen_yaw(t):
    return dict(roll=np.zeros(3), pitch=np.zeros(3), yawrate=np.full(3, 2.0 if 1.0 <= t < 2.5 else 0.0),
                zc=np.full(3, 1.0), thr=np.zeros(3))


ry = W.simulate(Py, scen_yaw, 4.0, dec=5)
yaw_rate = np.gradient(np.unwrap(np.radians(ry["yaw"]), axis=0), ry["t"][:, 0], axis=0)
out["yaw_2rads_step"] = [dict(cq=float(c), achieved_rate_rads=float(yaw_rate[(ry["t"][:, 0] > 2.0) & (ry["t"][:, 0] < 2.4), j].mean()),
                              t90_s=float(ry["t"][np.argmax(yaw_rate[:, j] > 1.8), j] - 1.0), max_tilt=float(ry["tilt"][:, j].max()))
                         for j, c in enumerate(cqs)]
print("  yaw:", out["yaw_2rads_step"])

# ======================================================================= T4  disturbance recovery
print("T4 recovery from a tilt / a bump")
tilts = np.array([20.0, 30.0, 45.0, 60.0, 75.0])
Nt = len(tilts)
Pt = W.make_params(Nt)
rt = W.simulate(Pt, W.const(N=Nt), 4.0, init=dict(roll=np.radians(tilts)), dec=5)
rec_rows = []
for j, a in enumerate(tilts):
    tt = rt["t"][:, j]
    ok = np.where(rt["tilt"][:, j] < 5)[0]
    rec_rows.append(dict(start_tilt_deg=a, level_within_5deg_s=round(float(tt[ok[0]]), 2) if len(ok) else None,
                         altitude_lost_cm=round(float((1.0 - rt["z"][:, j].min()) * 100), 0),
                         floor_hit=bool(rt["impacted"][j]), flipped=bool(rt["flipped"][j] and rt["tilt"][-1, j] > 20)))
out["recover_from_tilt_at_1m"] = rec_rows
for r in rec_rows:
    print("  ", r)
rates = np.array([4.0, 8.0, 12.0, 16.0])
Nk = len(rates)
Pk = W.make_params(Nk)
kicks = np.zeros((Nk, 3)); kicks[:, 0] = rates
rk = W.simulate(Pk, W.piecewise(Nk, [], kicks=[(1.0, kicks)]), 4.0, dec=5)
out["recover_from_bump"] = [dict(roll_rate_kick_rads=float(a), peak_tilt_deg=round(float(rk["tilt"][:, j].max()), 1),
                                 altitude_lost_cm=round(float((1.0 - rk["z"][:, j].min()) * 100), 0),
                                 floor_hit=bool(rk["impacted"][j])) for j, a in enumerate(rates)]
print("  ", out["recover_from_bump"])

# ======================================================================= T5  thrust x battery sweep (pass / fail map)
print("T5 thrust-per-motor x battery-charge sweep")
tm_list = [14, 16, 18, 20, 22, 25, 28, 32]
soc_list = [1.0, 0.6, 0.3, 0.1]
combos = [(tm, s) for s in soc_list for tm in tm_list]


def scen_stress(N):
    def f(t):
        on = 4.0 <= t < 5.5
        k = None
        return dict(roll=np.radians(20.0) * on * np.ones(N), pitch=np.zeros(N), yawrate=np.zeros(N),
                    zc=np.full(N, 1.0), thr=np.zeros(N))
    return f


def passfail(rec, i):
    ok = (not rec["flipped"][i]) and (not rec["impacted"][i]) and rec["z"][:, i].min() > 0.45 and rec["tilt"][-1, i] < 10
    return bool(ok)


sweep = {}
for mx, label in ((0.0, "56.6 g (5 g PCB)"), (8.5, "65.1 g (+15 % contingency)")):
    Ns = len(combos)
    Ps = W.make_params(Ns, mass_extra_g=mx, tmax_g=np.array([c[0] for c in combos], float),
                       soc=np.array([c[1] for c in combos], float))
    kk = np.zeros((Ns, 3)); kk[:, 0] = 4.0
    base = scen_stress(Ns)
    done = [False]

    def scen(t, base=base, kk=kk, done=done):
        d = base(t)
        if t >= 2.0 and not done[0]:
            d["kick"] = kk; done[0] = True
        return d
    rs = W.simulate(Ps, scen, 8.0, dec=10)
    grid = np.zeros((len(soc_list), len(tm_list)), bool)
    sat = np.zeros_like(grid, float)
    for i, (tm, s) in enumerate(combos):
        grid[soc_list.index(s), tm_list.index(tm)] = passfail(rs, i)
        sat[soc_list.index(s), tm_list.index(tm)] = rs["sat"][:, i].mean()
    sweep[label] = dict(grid=grid, sat=sat)
    print("  ", label)
    print("     thrust/motor g:", tm_list)
    for si, s in enumerate(soc_list):
        print("     charge %3.0f%%:" % (s * 100), " ".join("PASS" if x else "fail" for x in grid[si]))
out["thrust_sweep"] = {lab: dict(thrust_g=tm_list, charge=soc_list, pass_grid=v["grid"].tolist()) for lab, v in sweep.items()}

# ======================================================================= T6  flight time (battery drain, ToF hold + leans)
print("T6 flight time to cut-off")
W.DT = 0.005                                       # slower step for the long run
tm_e = [20, 25, 30]
cq_e = [0.0045, 0.0055, 0.0075]
mx_e = [0.0, 8.5]
combo_e = [(a, b, c) for a in tm_e for b in cq_e for c in mx_e]
Ne = len(combo_e)
Pe = W.make_params(Ne, mass_extra_g=np.array([c[2] for c in combo_e]), tmax_g=np.array([c[0] for c in combo_e], float),
                   cq=np.array([c[1] for c in combo_e]), n_delay=np.ones(Ne))


def scen_endurance(t):
    lean = 10.0 if (t % 12.0) < 2.0 and t > 4 else 0.0     # gentle 10 deg lean 2 s out of every 12
    ph = int(t // 12) % 2
    return dict(roll=np.radians(lean) * (ph == 0) * np.ones(Ne), pitch=np.radians(lean) * (ph == 1) * np.ones(Ne),
                yawrate=np.zeros(Ne), zc=np.full(Ne, 1.0), thr=np.zeros(Ne))


t_run = time.time()
re = W.simulate(Pe, scen_endurance, 520.0, dec=20, init=dict(z=1.0))
print("   endurance run %.0f s of compute" % (time.time() - t_run))
te = re["t"][:, 0]
flight = []
for i, (tm, cq, mx) in enumerate(combo_e):
    v = re["V"][:, i]; duty = re["duty_max"][:, i]; soc = re["soc"][:, i]
    hov_duty = np.convolve(duty, np.ones(40) / 40, mode="same")      # 4 s average
    idx_v = np.where((te > 3) & (v < 3.2))[0]
    idx_d = np.where((te > 3) & (hov_duty > 0.90))[0]
    idx_s = np.where(soc < 0.0)[0]
    t_v = te[idx_v[0]] if len(idx_v) else np.inf
    t_d = te[idx_d[0]] if len(idx_d) else np.inf
    t_s = te[idx_s[0]] if len(idx_s) else np.inf
    t_use = min(t_v, t_d, t_s)
    # hover current averaged
    flight.append(dict(thrust_g=tm, cq=cq, mass_g=round((Pe["mass"][i]) * 1000, 1), t_volt_3p2_min=round(t_v / 60, 1) if np.isfinite(t_v) else None,
                       t_headroom_min=round(t_d / 60, 1) if np.isfinite(t_d) else None,
                       t_empty_min=round(t_s / 60, 1) if np.isfinite(t_s) else None,
                       usable_min=round(t_use / 60, 1) if np.isfinite(t_use) else None,
                       hover_current_A=round(float(np.median(re["I"][(te > 5) & (te < 40), i])), 2),
                       hover_gW=round(float(Pe["mass"][i] * 1000 / (np.median(re["I"][(te > 5) & (te < 40), i]) * np.median(re["V"][(te > 5) & (te < 40), i]))), 1),
                       flipped=bool(re["flipped"][i]), floor_hit=bool(re["impacted"][i])))
out["flight_time"] = flight
W.DT = 0.002
for f in flight:
    print("  ", f)

# ======================================================================= T7  Monte Carlo
print("T7 Monte Carlo (varying everything I am unsure about)")
Nm = 600
rng = np.random.default_rng(11)
mc = dict(tmax_g=rng.uniform(16, 34, Nm), cq=rng.uniform(0.0045, 0.0075, Nm), Rm=rng.uniform(0.9, 1.8, Nm),
          Rb=rng.uniform(0.07, 0.16, Nm), Jr=rng.uniform(3e-8, 1.0e-7, Nm), n_delay=rng.integers(1, 5, Nm).astype(float),
          gyro_noise=rng.uniform(0.02, 0.08, Nm), acc_noise=rng.uniform(0.3, 1.0, Nm),
          dkt=np.clip(rng.normal(0, 0.07, (Nm, 4)), -0.15, 0.15), cg_dx=rng.normal(0, 0.0015, Nm),
          cg_dy=rng.normal(0, 0.0015, Nm), mass_err=rng.uniform(-0.08, 0.08, Nm),
          soc=rng.uniform(0.15, 1.0, Nm), cd=rng.uniform(0.015, 0.06, Nm), gyro_bias=np.abs(rng.normal(0, 0.003, Nm)))
extra = rng.uniform(0, 8.5, Nm)
Pm = W.make_params(Nm, mass_extra_g=extra, **mc)
axis = rng.integers(0, 3, Nm)
kk = np.zeros((Nm, 3)); kk[np.arange(Nm), axis] = rng.choice([-1, 1], Nm) * rng.uniform(2, 6, Nm)
rr_dir = rng.choice([-1, 1], Nm); pp_dir = rng.choice([-1, 1], Nm)
done = [False]


def scen_mc(t):
    d = dict(roll=np.radians(15) * rr_dir * (4.0 <= t < 5.5), pitch=np.radians(15) * pp_dir * (7.0 <= t < 8.5),
             yawrate=np.zeros(Nm), zc=np.full(Nm, 1.0), thr=np.zeros(Nm))
    if t >= 2.0 and not done[0]:
        d["kick"] = kk; done[0] = True
    return d


rm_ = W.simulate(Pm, scen_mc, 11.0, dec=10)
ok = np.array([passfail(rm_, i) for i in range(Nm)])
twr_cal = 4 * mc["tmax_g"] / (Pm["mass"] * 1000)
bins = [(0, 1.4), (1.4, 1.6), (1.6, 1.8), (1.8, 2.0), (2.0, 2.4), (2.4, 5)]
mcrows = []
for lo, hi in bins:
    sel = (twr_cal >= lo) & (twr_cal < hi)
    if sel.sum():
        mcrows.append(dict(thrust_to_weight_at_calibration=f"{lo}-{hi}", runs=int(sel.sum()), pass_pct=round(float(100 * ok[sel].mean()), 0)))
out["monte_carlo"] = dict(runs=Nm, overall_pass_pct=round(float(100 * ok.mean()), 0), by_twr=mcrows,
                          why_fail=dict(flipped=int(rm_["flipped"].sum()), floor_hit=int(rm_["impacted"].sum()),
                                        low=int((rm_["z"].min(axis=0) <= 0.45).sum())))
print("  overall pass %.0f %%" % (100 * ok.mean()))
for r in mcrows:
    print("  ", r)
# which parameter matters: failures among TWR>=1.8 only
sel = twr_cal >= 1.8
if sel.sum() and (~ok[sel]).sum():
    print("   failures with TWR>=1.8:", int((~ok[sel]).sum()), "of", int(sel.sum()))
    fl = ~ok & sel
    for nm, arr in (("n_delay", mc["n_delay"]), ("Jr", mc["Jr"]), ("soc", mc["soc"]), ("Rb", mc["Rb"]), ("mass extra g", extra)):
        print("    mean %s: failed %.3g vs passed %.3g" % (nm, arr[fl].mean(), arr[sel & ok].mean()))

json.dump(out, open("results.json", "w"), indent=1, default=float)

# ======================================================================= figures
fig, ax = plt.subplots(2, 3, figsize=(13, 7))
a = ax[0, 0]
a.plot(t2, z2, label="altitude (m)"); a.plot(t2, r2["tilt"][:, 0] / 10, label="tilt (deg / 10)"); a.plot(t2, r2["duty_max"][:, 0], label="max motor duty")
a.set_title("T2 takeoff, ToF altitude hold"); a.set_xlabel("s"); a.legend()
a = ax[0, 1]
for j, an in enumerate(angs):
    a.plot(rl["t"][:, j], rl["z"][:, j], label=f"{an:.0f} deg lean")
a.set_title("T3 altitude during 3 s pitch lean (1 s to 4 s)"); a.set_xlabel("s"); a.set_ylabel("m"); a.legend()
a = ax[0, 2]
for j, an in enumerate(angs):
    a.plot(rl["t"][:, j], rl["duty_max"][:, j], label=f"{an:.0f} deg")
a.axhline(1.0, color="k", ls="--"); a.set_title("T3 busiest motor duty (1.0 = no thrust left)"); a.set_xlabel("s"); a.legend()
a = ax[1, 0]
for j, an in enumerate(tilts):
    a.plot(rt["t"][:, j], rt["tilt"][:, j], label=f"start {an:.0f} deg")
a.set_title("T4 recovery from tilt at 1 m"); a.set_xlabel("s"); a.set_ylabel("tilt deg"); a.legend()
a = ax[1, 1]
for j, an in enumerate(rates):
    a.plot(rk["t"][:, j], rk["tilt"][:, j], label=f"{an:.0f} rad/s bump")
a.set_title("T4 recovery from a roll-rate bump"); a.set_xlabel("s"); a.legend()
a = ax[1, 2]
a.plot(r1["x"][:, 0], r1["y"][:, 0], label="nominal 30 s")
a.set_title("T1 horizontal drift in hover, nominal run"); a.set_xlabel("x m"); a.set_ylabel("y m"); a.axis("equal")
plt.tight_layout(); plt.savefig("wasp_sim_fig1.png", dpi=110); plt.close()

fig, ax = plt.subplots(1, 4, figsize=(15, 3.8))
for k_, (lab, v) in enumerate(sweep.items()):
    a = ax[k_]
    a.imshow(v["grid"].astype(float), cmap="RdYlGn", vmin=0, vmax=1, aspect="auto")
    a.set_xticks(range(len(tm_list))); a.set_xticklabels(tm_list); a.set_yticks(range(len(soc_list)))
    a.set_yticklabels([f"{int(s*100)}%" for s in soc_list]); a.set_xlabel("thrust per motor, full throttle, fresh pack (g)")
    a.set_ylabel("battery charge"); a.set_title("T5 survives bump + 20 deg lean?\n" + lab, fontsize=8); a.grid(False)
    for si in range(len(soc_list)):
        for ti in range(len(tm_list)):
            a.text(ti, si, "ok" if v["grid"][si, ti] else "X", ha="center", va="center", fontsize=7)
a = ax[2]
for i in range(0, Ne, 1):
    if combo_e[i][1] == 0.0055:
        a.plot(te / 60, re["V"][:, i], lw=1, label=f"{combo_e[i][0]} g, {Pe['mass'][i]*1000:.0f} g")
a.axhline(3.2, color="k", ls="--"); a.set_title("T6 battery volts under load while hovering"); a.set_xlabel("min"); a.legend(fontsize=6)
a = ax[3]
a.bar([r["thrust_to_weight_at_calibration"] for r in mcrows], [r["pass_pct"] for r in mcrows], color="tab:blue")
a.set_ylim(0, 100); a.set_ylabel("% runs OK"); a.set_xlabel("4 x thrust per motor / weight"); a.set_title(f"T7 Monte Carlo, {Nm} runs");
plt.tight_layout(); plt.savefig("wasp_sim_fig2.png", dpi=110); plt.close()
print("done")
