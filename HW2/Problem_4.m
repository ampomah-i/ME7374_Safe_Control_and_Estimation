% System parameters
m = 1;
c = 1.5;
k = 1.2;
Ts = 0.01; % Sampling time 

% Initial conditions and covariance matrices
x0 = [1; 0]; % Initial state
Q0 = 1000 * eye(2); % Initial covariance for state
Q_process = [0, 0; 0, 0.2]; % Process noise covariance
R_measure = 1e-4; % Measurement noise covariance

% Number of iterations
num_steps = 1000; % 10 seconds / 0.01s time steps
num_runs = 50; % Number of filter runs for averaging

% Storage for state and estimate history
x_true_history = zeros(2, num_steps, num_runs);
x_est_history = zeros(2, num_steps, num_runs);

% Define system dynamics function
f = @(x, w) [x(1) + Ts * x(2); ...
             x(2) + Ts * (-2*(c/m)*((x(1))^2 - 1)*x(2) - (k/m)*x(1) + w)];

% Kalman filter loop
for run = 1:num_runs
    % Initial state and covariance
    x_true = x0;
    x_est = x0;
    P_est = Q0;

    % Storage for each run
    x_true_run = zeros(2, num_steps);
    x_est_run = zeros(2, num_steps);

    for k = 1:num_steps
        % True dynamics with process noise
        w = sqrt(0.2) * randn;
        x_true = f(x_true, w);
        x_true_run(:, k) = x_true;

        % Measurement with noise
        v = sqrt(1e-4) * randn; % Measurement noise
        y = [1 0] * x_true + v;

        % Prediction step of EKF
        x_pred = f(x_est, 0);

        % Linearization (Jacobian of the process model at the estimated state)
        F = [1, Ts;
             -Ts * (2 * (c / m) * x_est(1) * x_est(2) + k / m), 1 - Ts * (c / m) * ((x_est(1))^2 - 1)];

        % Predict covariance
        P_pred = F * P_est * F' + Q_process;

        % Measurement model and linearization
        H = [1, 0];
        K = P_pred * H' / (H * P_pred * H' + R_measure);

        % Correction step of EKF
        x_est = x_pred + K * (y - H * x_pred);
        P_est = (eye(2) - K * H) * P_pred;

        % Store the estimated state
        x_est_run(:, k) = x_est;
    end

    % Store results of each run
    x_true_history(:, :, run) = x_true_run;
    x_est_history(:, :, run) = x_est_run;
end

% Calculate average over runs
x_true_avg = mean(x_true_history, 3);
x_est_avg = mean(x_est_history, 3);

% Plot results
time = (0:num_steps-1) * Ts;

figure;
subplot(2, 1, 1);
plot(time, x_true_avg(1, :), 'b', 'DisplayName', 'True x_1');
hold on;
plot(time, x_est_avg(1, :), 'r--', 'DisplayName', 'Estimated x_1');
xlabel('Time (s)');
ylabel('x_1');
legend;
title('Average State $x_1$ vs Estimated State $\hat{x}_1$', 'Interpreter', 'latex');

subplot(2, 1, 2);
plot(time, x_true_avg(2, :), 'b', 'DisplayName', 'True x_2');
hold on;
plot(time, x_est_avg(2, :), 'r', 'DisplayName', 'Estimated x_2');
xlabel('Time (s)');
ylabel('x_2');
legend;
title('Average State $x_2$ vs Estimated State $\hat{x}_2$', 'Interpreter', 'latex');
