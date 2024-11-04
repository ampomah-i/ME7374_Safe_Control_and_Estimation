% Parameters
m = 1; c = 1.5; k = 1.2;
Ts = 0.01;  % Sampling period
Q = 0.2;    % Process noise covariance
R = 1e-4;   % Measurement noise covariance
num_steps = 1000;  % Number of time steps (10 seconds)
num_trials = 50;   % Number of trials for averaging

% Initial state and covariance
initial_state = [1; 0];    % True initial state
P0 = 1000 * eye(2);        % Initial covariance

% Allocate memory to store results for averaging
x_true_history_all = zeros(2, num_steps, num_trials);
x_est_history_all = zeros(2, num_steps, num_trials);

for trial = 1:num_trials
    % Initialize states for each trial
    x_true = initial_state;
    x_est = initial_state;
    P = P0;
    
    for k = 1:num_steps
        % True system dynamics with process noise
        w = sqrt(0.2) * randn(2,1);  % Process noise
        x_true(1) = x_true(1) + Ts * x_true(2);
        x_true(2) = x_true(2) + Ts * (-2 * (c / m) * (x_true(1)^2 - 1) * x_true(2) - (k / m) * x_true(1)) + w(2);
        
        % Measurement with noise
        v = sqrt(1e-4) * randn;  % Measurement noise
        y = x_true(1) + v;
        
        % EKF Prediction Step
        F = [1, Ts; -Ts * (2 * c * x_est(1) * x_est(2) + k / m), 1 - 2 * Ts * c * (x_est(1)^2 - 1) / m];
        x_pred = [x_est(1) + Ts * x_est(2);
                  x_est(2) + Ts * (-2 * (c / m) * (x_est(1)^2 - 1) * x_est(2) - (k / m) * x_est(1))];
        P_pred = F * P * F' + Q * eye(2);
        
        % EKF Update Step
        H = [1, 0];  % Measurement matrix
        K = P_pred * H' / (H * P_pred * H' + R);  % Kalman gain
        x_est = x_pred + K * (y - H * x_pred);    % State update
        P = (eye(2) - K * H) * P_pred;            % Covariance update
        
        % Store results for this step
        x_true_history_all(:, k, trial) = x_true;
        x_est_history_all(:, k, trial) = x_est;
    end
end

% Average results across trials
x_true_mean = mean(x_true_history_all, 3);
x_est_mean = mean(x_est_history_all, 3);

% Plot averaged results
time = (0:num_steps-1) * Ts;
figure;
subplot(2,1,1);
plot(time, x_true_mean(1,:), 'b', time, x_est_mean(1,:), 'r--');
legend('True x_1 (Average)', 'Estimated x_1 (Average)');
xlabel('Time (s)'); ylabel('Position x_1');
title('Averaged True vs Estimated x_1');

subplot(2,1,2);
plot(time, x_true_mean(2,:), 'b', time, x_est_mean(2,:), 'r--');
legend('True x_2 (Average)', 'Estimated x_2 (Average)');
xlabel('Time (s)'); ylabel('Velocity x_2');
title('Averaged True vs Estimated x_2');
