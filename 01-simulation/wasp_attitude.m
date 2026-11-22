%% WASP Phase 1 - roll / pitch (attitude) model and tilt controller  [MATLAB]
%
% HOW TO RUN: open in MATLAB, press Run. It prints a table and opens 4 figures
% (one per test). Needs MATLAB R2018b or newer.
%
% THE IDEA
%   The four motors sit at the corners of a square. If the left motors push
%   harder than the right motors, the drone rolls. The controller:
%     1. MEASURES the tilt with a gyro (fast but drifts) and an accelerometer
%        (slow but doesn't drift). A "complementary filter" blends the two.
%     2. COMPARES the tilt with the tilt you asked for.
%     3. DECIDES how much extra thrust each motor should give (the "mixer").
%   Two loops, one inside the other:
%     outer loop (angle):  angle error  -> "how fast should I be rotating?"
%     inner loop (rate):   rate error   -> "how hard should I twist?" (torque)
%
% THE FOUR TESTS  (each draws 4 graphs: roll angle, roll rate, motor throttle, pitch)
%   1  Clean roll step      : ask for 10 degrees at 0.5 s, back to 0 at 2.0 s.
%   2  Battery 5 mm off-centre : a constant sideways twist, like a badly placed battery.
%   3  Weak motor            : rear-right motor only makes 85% of the thrust.
%   4  Gyro drift            : accelerometer help switched OFF.
%
% WHAT TO LOOK FOR  (tell me what you see, in your own words is fine)
%   Roll angle graph : does the blue line follow the dashed command? How long to
%                      get there? Does it overshoot? Is there a gap at the end?
%                      Is the orange (estimate) line the same as the blue (truth)?
%   Roll rate graph  : spikes = the drone twisting quickly. Should settle to zero.
%   Motor throttle   : lines that are NOT on top of each other show which motors
%                      are working harder to fix a problem. Anything touching the
%                      red dotted lines is a motor out of room (saturated).
%   Pitch graph      : should stay near zero. If it moves, something is coupling.
%
% SIMPLIFICATIONS (we remove these in later phases)
%   - Altitude is not simulated: total thrust is held at hover.
%   - No yaw (spin) yet.
%   - Angles stay small (< 20 degrees), so sin(x) is treated as x.
%   - The accelerometer is assumed to see the tilt. In a real flying drone it
%     mostly can't, because thrust always points along the drone's own "up".
%     Test 4 shows what happens without that help.
%
% THINGS TO TRY
%   - gains(): double Kp, then halve it. Which one makes it wobble?
%   - GYRO_BIAS and K_ACC in sensor_numbers(): how big can the bias get?
%   - Test 3: make the weak motor 0.6. Do any motors hit the red lines?
%   - IXX in airframe(): the 1.6e-4 is a guess. Try 1.0e-4 and 2.5e-4.
%
% NOTE: ported from a Python version that I ran and checked. This MATLAB file
% has NOT been run yet, so paste me any error message you get.

clear; clc; close all;

STEP = @(t) (t >= 0.5 && t < 2.0) * (10*pi/180);     % 10 degrees, from 0.5 s to 2.0 s
af = airframe();

tests = {
  struct('name','1 Clean roll step',                    'scale',[1 1 1 1],   'dist',[0 0],                      'k_acc',2.0)
  struct('name','2 Battery 5 mm off-centre',            'scale',[1 1 1 1],   'dist',[af.mass*9.81*0.005 0],     'k_acc',2.0)
  struct('name','3 Weak motor (rear-right at 85%)',     'scale',[1 1 1 0.85],'dist',[0 0],                      'k_acc',2.0)
  struct('name','4 Gyro drift, accelerometer help off', 'scale',[1 1 1 1],   'dist',[0 0],                      'k_acc',0.0)
};

fprintf('%-40s%11s%11s%14s%20s%18s\n','test','rise 10-90','overshoot','settle +-1deg','angle error @5-6 s','motors saturated');
for i = 1:numel(tests)
    tc = tests{i};
    o  = simulate(tc.scale, tc.dist, tc.k_acc, STEP, 6.0, 1);
    m  = step_metrics(o);
    tail = o.t >= 5.0;
    err  = mean(abs(o.phi(tail)))*180/pi;
    sat  = mean(any(o.duty >= 0.999 | o.duty <= 0.001, 2));
    fprintf('%-40s%8.0f ms%8.2f deg%11.0f ms%15.2f deg%13.1f %%\n', tc.name, ...
        m.rise*1000, m.over_deg, m.settle*1000, err, 100*sat);

    % ---- plots
    figure('Position',[80+30*i 80+30*i 1200 780]);
    t = o.t;
    subplot(2,2,1); hold on; grid on;
    plot(t, o.cmd*180/pi,  'k--');
    plot(t, o.phi*180/pi,  'b');
    plot(t, o.phih*180/pi, 'Color',[1 0.6 0.2]);
    title('Roll angle'); xlabel('time (s)'); ylabel('degrees'); legend('command','true roll','estimated roll');
    subplot(2,2,2); grid on;
    plot(t, o.p*180/pi); title('Roll rate'); xlabel('time (s)'); ylabel('degrees per second');
    subplot(2,2,3); hold on; grid on;
    plot(t, o.duty*100, 'LineWidth',0.9);
    yline(100,':r'); yline(0,':r');
    title('Motor throttle (red dotted = limits)'); xlabel('time (s)'); ylabel('duty %');
    legend('front-left','front-right','rear-left','rear-right','Location','east');
    subplot(2,2,4); hold on; grid on;
    plot(t, o.th*180/pi,  'b');
    plot(t, o.thh*180/pi, 'Color',[1 0.6 0.2]);
    title('Pitch angle (should stay near 0)'); xlabel('time (s)'); ylabel('degrees');
    legend('pitch angle','estimated pitch');
    sgtitle(['Test ' tc.name]);
end


%% ======================================================================
%  LOCAL FUNCTIONS
%  ======================================================================

function a = airframe()
a.mass = 0.1649;                 % kg, from the weight budget
a.D    = 0.0762;                 % prop diameter, m
a.A    = 0.060/sqrt(2);          % motor offset from centre in x and y (120 mm diagonal wheelbase)
a.IXX  = 1.6e-4;  a.IYY = 1.6e-4;   % kg m^2, ASSUMED. Fusion 360 can give the real value later.
% motor order: 1 front-left, 2 front-right, 3 rear-left, 4 rear-right  (x forward, y left)
a.POS_X = [+1 +1 -1 -1]*a.A;
a.POS_Y = [+1 -1 +1 -1]*a.A;
end

function m = motor_consts()
% same numbers as wasp_sim
m.KV = 3750; m.RM = 0.126; m.I0 = 0.6;
m.CT = 0.12; m.FOM = 0.44; m.J = 9e-7; m.RB = 0.04; m.VOC = 12.0;
m.KE = 60/(2*pi*m.KV);
end

function c = cq(ct, FOM)
% torque coefficient from thrust coefficient (momentum theory)
c = 0.7979 .* ct.^1.5 ./ FOM ./ (2*pi);
end

function g = gains()
g.Ka = 8.0;        % angle loop: wanted rate (rad/s) per rad of angle error
g.Kp = 30.0;       % rate loop P: angular acceleration per rad/s of rate error
g.Ki = 60.0;       % rate loop I
g.Kd = 0.004;      % rate loop D (on the measurement)
g.rate_max = 8.0;  % rad/s, limit on wanted rate
end

function s = sensor_numbers()
s.GYRO_NOISE     = 0.02;   % rad/s per sample, includes motor vibration   ASSUMED
s.GYRO_BIAS      = 0.01;   % rad/s, about 0.6 deg/s                        ASSUMED
s.ACC_TILT_NOISE = 0.03;   % rad, about 1.7 deg                            ASSUMED
end

function [duty_h, w_h] = hover_duty()
% duty each motor needs to hover, and the starting rotor speed
af = airframe(); mc = motor_consts(); RHO = 1.225; G = 9.81;
T = af.mass*G/4;
n = sqrt(T/(mc.CT*RHO*af.D^4));
w_h = 2*pi*n;
Q = cq(mc.CT, mc.FOM)*RHO*n^2*af.D^5;
I = Q/mc.KE + mc.I0;
Vb = mc.VOC - mc.RB*4*I;
duty_h = (mc.KE*w_h + I*mc.RM)/Vb;
end

function out = simulate(scale, dist, k_acc, roll_cmd, T, seed)
% scale : thrust multiplier per motor (1 = healthy, 0.85 = weak)
% dist  : constant disturbance torque [roll pitch] in N m
af = airframe(); mc = motor_consts(); gn = gains(); sn = sensor_numbers();
RHO = 1.225; G = 9.81;
rng(seed);
dt = 5e-4; every = 2; dtc = dt*every;       % physics 0.5 ms, controller 1 kHz
nsteps = round(T/dt);
scale = scale(:).';                          % row vector
ct_i = mc.CT*scale;
cq_i = cq(ct_i, mc.FOM);
pitch_cmd = @(t) 0.0;

[duty_h, w_h] = hover_duty();
w = w_h*ones(1,4);  duty = duty_h*ones(1,4);  Vb = mc.VOC;
phi = 0; p = 0; th = 0; q = 0;               % roll angle/rate, pitch angle/rate
phih = 0; thh = 0;                           % estimated angles
Ir = 0; Iq = 0;                              % integrators
pf = 0; qf = 0; dp = 0; dq = 0;              % filtered gyro rates and their derivatives
alpha_f = dtc/(dtc + 1/(2*pi*80));           % 80 Hz low-pass on the gyro
alpha_d = dtc/(dtc + 1/(2*pi*40));           % 40 Hz low-pass on the D term
Lcmd = 0; Mcmd = 0;

out.t = zeros(nsteps,1); out.phi = out.t; out.phih = out.t; out.p = out.t;
out.th = out.t; out.thh = out.t; out.cmd = out.t; out.Lcmd = out.t;
out.duty = zeros(nsteps,4); out.thrust = zeros(nsteps,4);

for k = 0:nsteps-1
    t = k*dt;
    % ---------------- physics: four motors ----------------
    n  = w/(2*pi);
    Q  = cq_i.*RHO.*n.^2.*af.D^5;
    I  = max((duty*Vb - mc.KE*w)/mc.RM, 0);
    Tm = ct_i.*RHO.*n.^2.*af.D^4;
    Vb = mc.VOC - mc.RB*sum(I);
    w  = max(w + dt*(mc.KE*(I - mc.I0*(w > 1)) - Q)/mc.J, 0);
    % ---------------- physics: tilt ----------------
    L = sum(af.POS_Y.*Tm);                   % roll torque: left motors minus right motors
    M = -sum(af.POS_X.*Tm);                  % pitch torque
    p = p + dt*(L + dist(1))/af.IXX;
    q = q + dt*(M + dist(2))/af.IYY;
    phi = phi + dt*p;
    th  = th  + dt*q;

    % ---------------- sensors + controller at 1 kHz ----------------
    if mod(k, every) == 0
        p_m = p + sn.GYRO_BIAS + sn.GYRO_NOISE*randn;
        q_m = q + sn.GYRO_BIAS + sn.GYRO_NOISE*randn;
        acc_r = phi + sn.ACC_TILT_NOISE*randn;
        acc_p = th  + sn.ACC_TILT_NOISE*randn;
        % complementary filter: integrate the gyro, nudge towards the accelerometer
        phih = phih + p_m*dtc + k_acc*(acc_r - phih)*dtc;
        thh  = thh  + q_m*dtc + k_acc*(acc_p - thh )*dtc;
        % filtered rate and its derivative
        pf_old = pf; qf_old = qf;
        pf = pf + alpha_f*(p_m - pf);
        qf = qf + alpha_f*(q_m - qf);
        dp = dp + alpha_d*((pf - pf_old)/dtc - dp);
        dq = dq + alpha_d*((qf - qf_old)/dtc - dq);
        % outer loop: angle error -> wanted rate
        p_ref = min(max(gn.Ka*(roll_cmd(t)  - phih), -gn.rate_max), gn.rate_max);
        q_ref = min(max(gn.Ka*(pitch_cmd(t) - thh ), -gn.rate_max), gn.rate_max);
        % inner loop: rate error -> angular acceleration -> torque
        er = p_ref - pf;  eq_ = q_ref - qf;
        Ir = min(max(Ir + gn.Ki*er *dtc, -300), 300);
        Iq = min(max(Iq + gn.Ki*eq_*dtc, -300), 300);
        ar = gn.Kp*er  + Ir - gn.Kd*dp;
        aq = gn.Kp*eq_ + Iq - gn.Kd*dq;
        Lcmd = af.IXX*ar;  Mcmd = af.IYY*aq;
        % mixer: split the torque request between the four motors
        T_cmd = af.mass*G/4 + (af.POS_Y*Lcmd - af.POS_X*Mcmd)/(4*af.A^2);
        T_cmd = max(T_cmd, 0);
        % thrust -> duty (inverse of the nominal motor model)
        n_c = sqrt(T_cmd/(mc.CT*RHO*af.D^4));
        w_c = 2*pi*n_c;
        Ic  = cq(mc.CT, mc.FOM)*RHO*n_c.^2*af.D^5/mc.KE + mc.I0;
        duty = min(max((mc.KE*w_c + Ic*mc.RM)/Vb, 0), 1);
    end

    out.t(k+1) = t;  out.phi(k+1) = phi;  out.phih(k+1) = phih;  out.p(k+1) = p;
    out.th(k+1) = th; out.thh(k+1) = thh; out.cmd(k+1) = roll_cmd(t);
    out.duty(k+1,:) = duty;  out.thrust(k+1,:) = Tm/G*1000;  out.Lcmd(k+1) = Lcmd;
end
end

function m = step_metrics(o)
t0 = 0.5; target = 10*pi/180; band = 1*pi/180;
t = o.t; phi = o.phi;
after = t >= t0;
r10 = first_reach(t, after, phi, 0.1*target, t0);
r90 = first_reach(t, after, phi, 0.9*target, t0);
m.rise = r90 - r10;
win = (t >= t0) & (t < 2.0);                 % judge on the first step only
out = find(win & abs(phi - target) > band);
if isempty(out), m.settle = 0; else, m.settle = t(out(end)) - t0; end
m.over_deg = max(max(phi(win)) - target, 0)*180/pi;
end

function tt = first_reach(t, after, x, level, t0)
idx = find(after & x >= level, 1, 'first');
if isempty(idx), tt = inf; else, tt = t(idx) - t0; end
end
