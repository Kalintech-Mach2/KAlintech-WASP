%% WASP Phase 1 - physics model and altitude step test (MATLAB version)
%
% HOW TO RUN: open this file in MATLAB and press Run. It prints a report in
% the Command Window and draws the plots. Needs MATLAB R2018b or newer
% (local functions at the bottom of a script, plus yline/xline).
%
% WHAT IT DOES
%   1. Weight budget
%   2. Motor + prop model   (YSIDO 1505 3750KV, 3S, Gemfan 3016)
%   3. 0 m -> 1 m altitude step, settle within +-2 cm, for 3 sensor types
%
% LABELS IN THE COMMENTS
%   SELLER  = number from a datasheet / seller page
%   ASSUMED = my estimate, replace it with a measured value when you can
%
% THINGS TO TRY (change one number, run again, watch what moves)
%   - nominal(): Ct from 0.12 to 0.07. This is the thrust stand number.
%     Everything about thrust-to-weight and hover time hangs on it.
%   - default_ctrl(): vmax 1.5 -> 3. Watch the overshoot come back.
%   - sensors(): ToF sigma 0.006 -> 0.02. Watch the +-2 cm band get missed.
%   - run_one(...,'brake',true) in the main section: ESC active braking.
%
% NOTE: this was ported line by line from a Python version that I ran and
% checked. It has NOT been run in MATLAB yet, so if you get an error message,
% paste it to me and I will fix it.

clear; clc; close all;

physics_report();

% ---- fastest physically possible 0 -> 1 m (full thrust up, zero thrust to brake)
RHO = 1.225; G = 9.81;
p    = nominal();
fullThr = steady_state(1.0, 12.0, p.Ct, p.Kv);
a_up = 4*fullThr.T_N/p.mass - G;
a_dn = G;
v2   = 1/(1/(2*a_up) + 1/(2*a_dn));
t_min = sqrt(v2)/a_up + sqrt(v2)/a_dn;
fprintf('%s\n', repmat('=',1,66));
fprintf('PHYSICAL LIMIT: climb accel %.1f m/s^2, fastest possible 0->1 m stop = %.2f s (ignores motor lag)\n', a_up, t_min);
fprintf('%s\n', repmat('=',1,66));

% ---- step test, nominal parameters, one run per sensor
S = sensors();
fprintf('STEP TEST: 0 -> 1 m, nominal parameters, one run per sensor\n');
fprintf('  %-27s%9s%11s%14s%16s%8s\n','sensor','t to 90%','overshoot','settle +-2cm','rms (last 2 s)','peak');
runs = cell(1,numel(S));
for i = 1:numel(S)
    runs{i} = run_one(S(i), 1, 'spread',false);
    m = metrics(runs{i});
    if isinf(m.settle(1)), st = 'never'; else, st = sprintf('%.2f s', m.settle(1)); end
    fprintf('  %-27s%7.2f s%8.1f cm%14s%13.1f mm%6.0f mm\n', S(i).name, m.t90(1), ...
        m.overshoot(1)*100, st, m.rms_tail(1)*1000, m.peak_tail(1)*1000);
end

fprintf('  (ESC passive coast-down assumed. With Bluejay active braking enabled:)\n');
for i = 1:2
    mb = metrics(run_one(S(i), 1, 'spread',false, 'brake',true));
    fprintf('  %-27s%7.2f s%8.1f cm%11.2f s%13.1f mm%6.0f mm\n', S(i).name, mb.t90(1), ...
        mb.overshoot(1)*100, mb.settle(1), mb.rms_tail(1)*1000, mb.peak_tail(1)*1000);
end

% ---- robustness: 300 runs each with random parameters
fprintf('%s\n', repmat('=',1,66));
fprintf('ROBUSTNESS: 300 runs each, mass +-10%%, Ct +-20%%, Kv +-5%%, battery 11.4-12.6 V, accel bias\n');
mc = cell(1,numel(S));
for i = 1:numel(S)
    o  = run_one(S(i), 300, 'spread',true, 'seed',7);
    m  = metrics(o);
    mc{i} = m;
    good = (m.peak_tail <= 0.02) & isfinite(m.settle);
    st   = m.settle(isfinite(m.settle));
    if isempty(st), med = NaN; else, med = median(st); end
    sp   = sort(m.peak_tail);  p95 = sp(ceil(0.95*numel(sp)))*1000;   % 95th percentile
    fprintf('  %-27s within +-2 cm after settling: %5.1f%%   median settle %4.2f s   95th pct peak error %6.1f mm\n', ...
        S(i).name, 100*mean(good), med, p95);
end

% ---- plots
cols = [0.12 0.47 0.71; 0.17 0.63 0.17; 0.84 0.15 0.16];
figure('Position',[100 100 1250 800]);
subplot(2,2,1); hold on; grid on;
for i = 1:numel(S), plot(runs{i}.t, runs{i}.z(:,1), 'Color',cols(i,:), 'DisplayName',S(i).name); end
yline(1.0,':k'); title('0 to 1 m step (command at 0.5 s)'); xlabel('time (s)'); ylabel('altitude (m)');
legend('Location','southeast');

subplot(2,2,2); hold on; grid on;
patch([0.5 6 6 0.5],[0.98 0.98 1.02 1.02],[1 0.84 0],'FaceAlpha',0.35,'EdgeColor','none','DisplayName','+-2 cm band');
for i = 1:numel(S), plot(runs{i}.t, runs{i}.z(:,1), 'Color',cols(i,:), 'DisplayName',S(i).name); end
ylim([0.9 1.1]); xlim([0.5 6]); title('Zoom on the target band'); xlabel('time (s)'); ylabel('altitude (m)');

subplot(2,2,3); hold on; grid on;
for i = 1:numel(S), plot(runs{i}.t, runs{i}.duty(:,1)*100, 'Color',cols(i,:)); end
title('ESC duty (throttle) during the step'); xlabel('time (s)'); ylabel('duty (%)');

subplot(2,2,4); hold on; grid on;
for i = 1:numel(S)
    histogram(min(mc{i}.peak_tail*1000, 200), 40, 'FaceColor',cols(i,:), 'FaceAlpha',0.55, 'EdgeColor','none');
end
xline(20,'--k');
title('Worst hold error in last 2 s, 300 runs (dashed = 20 mm = +-2 cm)');
xlabel('peak error (mm), clipped at 200'); ylabel('runs');
legend({S.name},'Location','north');


%% ======================================================================
%  LOCAL FUNCTIONS (everything below is a helper the main section calls)
%  ======================================================================

function B = weight_budget()
% {name, grams, source}
B = {
 'Motors x4 (11 g each)',               44.0, 'SELLER'
 '4-in-1 ESC (JHEMCU EM40A)',            5.2, 'SELLER'
 'Battery 3S 650 mAh Tattu',            59.0, 'SELLER, +-10 to 20 g tolerance'
 'Props x4 (Gemfan 3016, ~1.2 g)',       4.8, 'ASSUMED'
 'ESP32-S3 SuperMini',                   2.5, 'ASSUMED'
 'MPU-6050 GY-521',                      2.1, 'ASSUMED'
 '5 V buck regulator',                   1.3, 'ASSUMED'
 'Custom PCB + headers',                 6.0, 'ASSUMED'
 'Wiring, XT30 pigtail, connectors',     6.0, 'ASSUMED'
 'Frame (arms + plates), design target',22.0, 'ASSUMED, design to this'
 'Screws, standoffs, ties',              5.0, 'ASSUMED'
 'Battery strap',                        3.0, 'ASSUMED'
 'Solder, tape, misc',                   2.0, 'ASSUMED'
 'Laser ToF sensor (VL53L1X)',           2.0, 'ASSUMED'
};
end

function p = nominal()
B = weight_budget();
p.D    = 0.0762;        % prop diameter, m (3 in)
p.Kv   = 3750;          % rpm per volt                         SELLER
p.Rm   = 0.096 + 0.03;  % ohm: motor 96 mOhm (SELLER) + ESC FET and wire (ASSUMED)
p.I0   = 0.6;           % A, no-load / friction current         ASSUMED
p.Ct   = 0.12;          % thrust coeff, T = Ct*rho*n^2*D^4      ASSUMED (main unknown)
p.FoM  = 0.44;          % figure of merit, calibrated on the seller test (631 g @ 288 W)
p.J    = 9e-7;          % kg m^2, rotor + prop inertia          ASSUMED
p.Rb   = 0.04;          % ohm, battery internal resistance      ASSUMED
p.Voc  = 12.0;          % V, charged pack under light load
p.CdA  = 0.0144;        % m^2, drag area for vertical motion    ASSUMED
p.mass = sum(cell2mat(B(:,2)))/1000;   % kg
end

function c = default_ctrl()
c.Kz    = 3.0;    % position loop gain near the target (1/s)
c.Kv    = 12.0;   % velocity loop P gain (accel per m/s)
c.Ki    = 25.0;   % velocity loop I gain
c.vmax  = 1.5;    % climb speed limit (m/s)
c.amax  = 18.0;   % max commanded upward accel (m/s^2)
c.a_dec = 3.0;    % braking decel the position loop plans for (m/s^2)
end

function S = sensors()
% fs = sample rate Hz, sigma = noise m, delay = s, rw = random walk m/sqrt(s),
% w_est = estimator pole rad/s. These noise numbers are my ESTIMATES.
S(1) = struct('name','Mocap-grade (ideal)',      'fs',100,'sigma',0.002,'delay',0.010,'rw',0.000,'w_est',25.0,'ctrl',[]);
S(2) = struct('name','Laser ToF (VL53-class)',   'fs',50, 'sigma',0.006,'delay',0.020,'rw',0.000,'w_est',15.0,'ctrl',[]);
baro_ctrl = struct('Kz',0.8,'Kv',8.0,'Ki',3.0,'vmax',0.6,'amax',18.0,'a_dec',2.0);
S(3) = struct('name','Barometer (BMP/DPS-class)','fs',25, 'sigma',0.15, 'delay',0.040,'rw',0.030,'w_est',1.5,'ctrl',baro_ctrl);
end

function Cp = cp_from_ct(Ct, FoM)
% Momentum theory links the power and thrust coefficients.
Cp = 0.7979 .* Ct.^1.5 ./ FoM;
end

function r = steady_state(duty, Voc, Ct, Kv)
% Steady rotor speed, thrust and current at a given ESC duty (0..1).
p = nominal(); RHO = 1.225; G = 9.81;
Ke = 60/(2*pi*Kv);                       % V per rad/s, same number as N.m per A
Cq = cp_from_ct(Ct, p.FoM)/(2*pi);
Iof = @(w) (Cq*RHO*(w/(2*pi))^2*p.D^5)/Ke + p.I0;     % motor current needed at speed w
f   = @(w) duty*(Voc - p.Rb*4*Iof(w)) - (Ke*w + Iof(w)*p.Rm);
w   = fzero(f, [1, 2*pi*Kv*Voc/60]);
n   = w/(2*pi);
I   = Iof(w);
T   = Ct*RHO*n^2*p.D^4;
Vb  = Voc - p.Rb*4*I;
r.w = w;  r.rpm = 60*n;  r.T_N = T;  r.T_g = T/G*1000;  r.I = I;
r.P_in = Vb*I;  r.Vb = Vb;  r.I_total = 4*I;
end

function [duty, r] = hover_point(Ct, mass, Voc)
% ESC duty and current needed to hover.
p = nominal(); G = 9.81;
T_need = mass*G/4;
duty = fzero(@(d) thrust_N(d, Voc, Ct, p.Kv) - T_need, [0.01, 1.0]);
r = steady_state(duty, Voc, Ct, p.Kv);
end

function T = thrust_N(duty, Voc, Ct, Kv)
r = steady_state(duty, Voc, Ct, Kv);
T = r.T_N;
end

function physics_report()
p = nominal(); G = 9.81;
B = weight_budget();
fprintf('%s\n', repmat('=',1,66));
fprintf('WEIGHT BUDGET\n');
for i = 1:size(B,1)
    fprintf('  %-42s%6.1f g   [%s]\n', B{i,1}, B{i,2}, B{i,3});
end
tot = p.mass*1000;
fprintf('  %-42s%6.1f g\n', 'TOTAL', tot);
fprintf('  with +15%% contingency: %6.1f g   (requirement: < 250 g)\n', tot*1.15);
fprintf('%s\n', repmat('=',1,66));
fprintf('THRUST, CURRENT, THROTTLE  (Ct is the main unknown)\n');
fprintf('  %5s %11s %6s %11s %11s %8s %11s\n','Ct','Tmax/motor','T/W','Imax total','hover duty','hover I','hover time');
for Ct = [0.07 0.09 0.12 0.15]
    fullThr = steady_state(1.0, 12.0, Ct, p.Kv);
    [hd, hp] = hover_point(Ct, p.mass, 11.7);
    tw   = 4*fullThr.T_N/(p.mass*G);
    I_h  = hp.I_total + 0.25;            % plus electronics
    mins = 0.8*0.650/I_h*60;             % 80% usable capacity
    fprintf('  %5.2f %9.0f g %6.1f %9.1f A %9.0f %% %6.1f A %8.1f min\n', ...
        Ct, fullThr.T_g, tw, fullThr.I_total, hd*100, I_h, mins);
end
fprintf('  Battery limit: 75C x 0.65 Ah = 48.75 A continuous\n');
fprintf('  Motor limit (seller): 18 A / 200 W each\n');
end

function out = run_one(S, N, varargin)
% Wrapper: run_one(sensor, N, 'spread',true,'seed',7,'brake',true,'T',6)
opt = struct('spread',false,'seed',1,'brake',false,'T',6.0,'ctrl',default_ctrl());
for k = 1:2:numel(varargin), opt.(varargin{k}) = varargin{k+1}; end
out = simulate(S, N, opt);
end

function out = simulate(S, N, opt)
% Vertical-axis simulation. N runs are simulated at once (columns).
p = nominal(); RHO = 1.225; G = 9.81;
dt = 1e-3; ctrl_every = 2; t_step = 0.5; z_target = 1.0;
nsteps = round(opt.T/dt);
if ~isempty(S.ctrl), ctrl = S.ctrl; else, ctrl = opt.ctrl; end   % a sensor may bring its own gains

rng(opt.seed);
% "true" parameters of each run (the controller only knows the nominal ones)
if opt.spread
    U = @(lo,hi) lo + (hi-lo)*rand(N,1);
    abias = -0.05 + 0.10*rand(N,1);
else
    U = @(lo,hi) 0.5*(lo+hi)*ones(N,1);
    abias = zeros(N,1);
end
mass = p.mass*U(0.9,1.1);
Ct   = p.Ct  *U(0.8,1.2);
Kv   = p.Kv  *U(0.95,1.05);
Voc  = U(11.4,12.6);

Ke   = 60./(2*pi*Kv);
Cq   = cp_from_ct(Ct, p.FoM)/(2*pi);
Ke_n = 60/(2*pi*p.Kv);                    % nominal values used inside the controller
Cq_n = cp_from_ct(p.Ct, p.FoM)/(2*pi);

% states
z = zeros(N,1); v = zeros(N,1); w = zeros(N,1);
zh = zeros(N,1); vh = zeros(N,1); bh = zeros(N,1);
Iv = zeros(N,1); duty = zeros(N,1); zm = zeros(N,1); rwalk = zeros(N,1);
wc = S.w_est; k1 = 3*wc; k2 = 3*wc^2; k3 = wc^3;     % estimator: three equal poles
dtc = ctrl_every*dt;
sens_every = max(1, round(1/S.fs/dt));
delay_n    = round(S.delay/dt);

zhist = zeros(nsteps+1, N);
out.t = ((0:nsteps-1)*dt).';
out.z = zeros(nsteps,N); out.duty = zeros(nsteps,N);
out.zh = zeros(nsteps,N); out.I = zeros(nsteps,N);
Vb = Voc;

for k = 0:nsteps-1
    t = k*dt;
    if t >= t_step, zref = z_target; else, zref = 0; end

    % ---- physics: motor + prop (all four identical)
    n     = w/(2*pi);
    Q     = Cq.*RHO.*n.^2.*p.D^5;
    I_raw = (duty.*Vb - Ke.*w)/p.Rm;
    if opt.brake, I = I_raw; else, I = max(I_raw,0); end   % brake: ESC may brake the rotor
    I     = max(I,-12);                                    % braking current limit (A)
    Tm    = Ct.*RHO.*n.^2.*p.D^4;
    Ib_total = 4*max(I,0);                                 % braking current does not come from the battery
    Vb    = Voc - p.Rb*Ib_total;
    torque = Ke.*(I - p.I0*(w>1)) - Q;
    w     = max(w + dt*torque/p.J, 0);

    % ---- physics: vertical dynamics
    Fdrag = 0.5*RHO*p.CdA*v.*abs(v);
    F  = 4*Tm - mass*G - Fdrag;
    on_ground = (z <= 0) & (F <= 0);
    a  = F./mass;      a(on_ground) = 0;
    v  = v + a*dt;     v(on_ground) = 0;
    z  = max(z + v*dt, 0);  z(on_ground) = 0;
    zhist(k+2,:) = z.';

    % ---- sensor sample (position)
    if mod(k, sens_every) == 0
        idx   = max(k - delay_n, 0);
        rwalk = rwalk + S.rw*sqrt(sens_every*dt)*randn(N,1);
        zm    = zhist(idx+1,:).' + rwalk + S.sigma*randn(N,1);
    end

    % ---- estimator + controller at control rate
    if mod(k, ctrl_every) == 0
        % accelerometer: specific force plus noise (vibration included)
        f_spec = (4*Tm - Fdrag)./mass;  f_spec(on_ground) = G;
        a_meas = f_spec + abias + 0.30*randn(N,1) - G;
        e  = zm - zh;
        zh = zh + (vh + k1*e)*dtc;
        vh = vh + (a_meas - bh + k2*e)*dtc;
        bh = bh - k3*e*dtc;

        % square-root position controller: brakes early, lands softly on the target
        err = zref - zh;
        lin = ctrl.a_dec/ctrl.Kz^2;                       % below this error: plain P
        vsq = sqrt(2*ctrl.a_dec*max(abs(err) - 0.5*lin, 0));
        vcmd = ctrl.Kz*err;
        big  = abs(err) > lin;
        vcmd(big) = sign(err(big)).*vsq(big);
        vcmd = min(max(vcmd, -ctrl.vmax), ctrl.vmax);

        % velocity PI loop -> acceleration command
        ev   = vcmd - vh;
        acmd = ctrl.Kv*ev + Iv;
        acmd_sat = min(max(acmd, -0.8*G), ctrl.amax);
        keep = (acmd == acmd_sat);                         % anti-windup: only integrate when not saturated
        Iv(keep) = Iv(keep) + ctrl.Ki*ev(keep)*dtc;
        Iv = min(max(Iv, -6), 6);

        % acceleration -> thrust -> rotor speed -> voltage -> duty (nominal model inverted)
        T_each = max(p.mass*(G + acmd_sat)/4, 0);
        n_c = sqrt(T_each/(p.Ct*RHO*p.D^4));
        w_c = 2*pi*n_c;
        Qc  = Cq_n*RHO*n_c.^2*p.D^5;
        Ic  = Qc/Ke_n + p.I0;
        Vt  = Ke_n*w_c + Ic*p.Rm;
        duty = min(max(Vt./Vb, 0), 1);
        if zref == 0, duty(z <= 0.001) = 0; end            % nothing wanted, still on the ground: motors off
    end

    out.z(k+1,:) = z.';  out.duty(k+1,:) = duty.';
    out.zh(k+1,:) = zh.'; out.I(k+1,:) = Ib_total.';
end
end

function m = metrics(out)
t_step = 0.5; target = 1.0; band = 0.02;
t = out.t; Z = out.z; N = size(Z,2);
after = t >= t_step;
m.t90 = zeros(1,N); m.t10_90 = zeros(1,N); m.overshoot = zeros(1,N);
m.settle = zeros(1,N); m.rms_tail = zeros(1,N); m.peak_tail = zeros(1,N);
tail = t >= t(end) - 2.0;
for i = 1:N
    zi = Z(:,i);
    t10 = first_reach(t, after, zi, 0.1*target, t_step);
    t90 = first_reach(t, after, zi, 0.9*target, t_step);
    m.t90(i) = t90;  m.t10_90(i) = t90 - t10;
    m.overshoot(i) = max(max(zi) - target, 0);
    outside = find(after & abs(zi - target) > band);
    if isempty(outside)
        m.settle(i) = 0;
    elseif outside(end) >= numel(t)
        m.settle(i) = inf;
    else
        m.settle(i) = t(outside(end)) - t_step;
    end
    m.rms_tail(i)  = sqrt(mean((zi(tail) - target).^2));
    m.peak_tail(i) = max(abs(zi(tail) - target));
end
end

function tt = first_reach(t, after, zi, level, t_step)
% time (since the step command) when the signal first reaches `level`
idx = find(after & zi >= level, 1, 'first');
if isempty(idx), tt = inf; else, tt = t(idx) - t_step; end
end
