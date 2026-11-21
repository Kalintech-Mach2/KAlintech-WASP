% 1-D hover model with PID control
m = 0.09; g = 9.81;              % drone mass (kg), gravity (undefeated)
Tmax = 3*m*g;                    % total thrust limit (3:1 thrust-to-weight)
dt = 0.002; t = 0:dt:6;          % time step and time vector
z = zeros(size(t)); v = zeros(size(t));
Kp = 8; Ki = 1; Kd = 1.5;        % PID gains: push, memory, braking
zref = 1; integ = 0;             % target height (m)

for k = 1:numel(t)-1
    e = zref - z(k);
    integ = integ + e*dt;
    u = m*g + Kp*e + Ki*integ - Kd*v(k);  % hover thrust + PID
    u = min(max(u, 0), Tmax);             % motors can't exceed their limits
    a = u/m - g;                          % F = ma
    v(k+1) = v(k) + a*dt;
    z(k+1) = z(k) + v(k+1)*dt;
end

plot(t, z); hold on; plot(t, zref*ones(size(t)), '--');
xlabel('time (s)'); ylabel('height (m)'); grid on
