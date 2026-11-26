"""Tune the attitude loop gains on a small set of 'bad-ish' vehicles at once, so the result is not
tuned only for the perfect case. Prints gains to paste into GAINS in wasp_micro_sim.py."""
import numpy as np, sys, time
import wasp_micro_sim as W
from scipy.optimize import minimize

def variants():
    return W.make_params(3, Jr=np.array([5e-8, 9e-8, 5e-8]), n_delay=np.array([2, 3, 4.0]),
                         tmax_g=np.array([25, 22, 20.0]), gyro_noise=np.array([0.03, 0.05, 0.05]))

def cost(x, verbose=False):
    Ka, Kp, Ki, Kd = np.exp(x)
    if Ka > 20 or Kp > 80 or Kd > 0.08 or Ki > 150:
        return 1e3
    P = variants()
    segs = [(0.5, 15, 0), (2.5, 0, 0)]
    r = W.simulate(P, W.piecewise(3, segs), 4.0, gains=dict(Ka=Ka, Kp=Kp, Ki=Ki, Kd=Kd), mode="alt")
    t = r['t'][:, 0]; roll = r['roll']
    m = (t > 0.5) & (t < 2.5)
    err = np.abs(roll[m] - 15).mean(axis=0)
    over = np.maximum(roll[m].max(axis=0) - 15, 0)
    # jitter in the motor commands during hover (noise amplification)
    h = t < 0.4
    jit = r['duty_max'][h].std(axis=0) + 0.0
    flips = r['flipped'].sum()
    c = (err / 15 + 0.06 * over / 15 * 15 / 5 + 8 * jit + 50 * flips).mean()
    if verbose:
        print("  err %s over %s jit %s" % (err.round(2), over.round(2), jit.round(4)))
    return c

x0 = np.log([4.0, 40.0, 25.0, 0.006])
t0 = time.time()
res = minimize(cost, x0, method="Nelder-Mead", options=dict(maxiter=110, xatol=0.02, fatol=1e-3))
Ka, Kp, Ki, Kd = np.exp(res.x)
print("cost start %.3f -> %.3f  (%.0f s)" % (cost(x0), res.fun, time.time() - t0))
print("GAINS: Ka=%.2f Kp=%.2f Ki=%.2f Kd=%.4f" % (Ka, Kp, Ki, Kd))
cost(res.x, verbose=True)
