A = [0.9719, 0.0155; 0.2097, 0.9705];
B = [0.0071; 0.3263];
C = [1, 0];
D = 0;

N = 200;

x0 = [10; 0];
x_target = [0; 0];

Q = diag([1, 1]);
R = 0.1;

[K, ~, ~] = dlqr(A, B, Q, R);

x = zeros(2, N+1);
u = zeros(1, N);
x(:, 1) = x0;

for k = 1:N
    u(k) = -K * (x(:, k) - x_target);
    u(k) = max(min(u(k), 25), -25);
    x(:, k+1) = A * x(:, k) + B * u(k);
    x(1, k+1) = max(min(x(1, k+1), 30), -15);
    x(2, k+1) = max(min(x(2, k+1), 100), -100);
end

time = 0:N;

figure;
sgtitle('LQR Control Results');
subplot(3, 1, 1);
plot(time, x(1, :), 'b', 'DisplayName', 'Angle of Elevation (\alpha)');
xlabel('Time Step');
ylabel('\alpha');
title('Angle of Elevation (\alpha) over Time');
legend;

subplot(3, 1, 2);
plot(time, x(2, :), 'r', 'DisplayName', 'Pitch Angular Rate (q)');
xlabel('Time Step');
ylabel('q');
title('Pitch Angular Rate (q) over Time');
legend;

subplot(3, 1, 3);
stairs(0:N-1, u, 'k', 'DisplayName', 'Control Input (\delta)');
xlabel('Time Step');
ylabel('\delta');
title('Control Input (\delta) over Time');
legend;

A = [0.9719, 0.0155; 0.2097, 0.9705];
B = [0.0071; 0.3263];
N = 200;
horizon = 10;

alpha_min = -15; alpha_max = 30;
q_min = -100; q_max = 100;
delta_min = -25; delta_max = 25;

Q = diag([10, 10]);
R = 1;

x0 = [10; 0];
x_target = [0; 0];

x_mpc = zeros(2, N+1);
u_mpc = zeros(1, N);
x_mpc(:, 1) = x0;

for i = 1:N
    x = sdpvar(2, horizon+1);
    u = sdpvar(1, horizon);
    
    objective = 0;
    constraints = [x(:, 1) == x_mpc(:, i)];

    for k = 1:horizon
        objective = objective + (x(:, k) - x_target)' * Q * (x(:, k) - x_target) + u(k)' * R * u(k);
        constraints = [constraints, x(:, k+1) == A * x(:, k) + B * u(k)];
        constraints = [constraints, alpha_min <= x(1, k) <= alpha_max];
        constraints = [constraints, q_min <= x(2, k) <= q_max];
        constraints = [constraints, delta_min <= u(k) <= delta_max];
    end

    objective = objective + (x(:, horizon+1) - x_target)' * Q * (x(:, horizon+1) - x_target);

    options = sdpsettings('solver', 'quadprog', 'verbose', 0);
    diagnostics = optimize(constraints, objective, options);

    if diagnostics.problem == 0
        u_mpc(i) = value(u(1));
        x_mpc(:, i+1) = A * x_mpc(:, i) + B * u_mpc(i);
        x_mpc(1, i+1) = max(min(x_mpc(1, i+1), alpha_max), alpha_min);
        x_mpc(2, i+1) = max(min(x_mpc(2, i+1), q_max), q_min);
    else
        warning('MPC optimization failed at step %d', i);
        break;
    end
end

time = 0:N;

figure;
sgtitle('MPC Control Results');
subplot(3, 1, 1);
plot(time, x_mpc(1, :), 'b', 'DisplayName', 'Angle of Elevation (\alpha)');
xlabel('Time Step');
ylabel('\alpha');
title('Angle of Elevation (\alpha) over Time');
legend;

subplot(3, 1, 2);
plot(time, x_mpc(2, :), 'r', 'DisplayName', 'Pitch Angular Rate (q)');
xlabel('Time Step');
ylabel('q');
title('Pitch Angular Rate (q) over Time');
legend;

subplot(3, 1, 3);
stairs(0:N-1, u_mpc, 'k', 'DisplayName', 'Control Input (\delta)');
xlabel('Time Step');
ylabel('\delta');
title('Control Input (\delta) over Time');
legend;
