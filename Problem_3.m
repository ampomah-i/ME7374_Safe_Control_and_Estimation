% Clear workspace and command window
clear; clc;

%% Standard Optimization

% Define decision variables
x = sdpvar(2, 1);        % Decision variables x1 and x2
zeta = sdpvar(2, 1);     % Auxiliary variables zeta1 and zeta2

% Define constraints
Constraints = [
    x(1) >= zeta(1), 
    x(2) >= zeta(2), 
    norm(zeta, 1) <= 1    % Using 1-norm for zeta
];

% Define objective function
Objective = x(1) + x(2);

% Set solver options
options = sdpsettings('solver', 'mosek', 'verbose', 1);

% Solve the optimization problem
sol = optimize(Constraints, Objective, options);

% Check solution status
if sol.problem == 0
    % Extract and display the optimal solution
    optimal_x = value(x);
    fprintf('Optimal solution for x:\n');
    disp(optimal_x);
else
    % Display error information
    disp('Solver failed to find an optimal solution.');
    disp(sol.info);
    yalmiperror(sol.problem);
end

%% Robust Optimization

% Clear variables for the robust optimization
clear x zeta Constraints Objective sol

% Define decision variables
x = sdpvar(2, 1);        % Decision variables x1 and x2

% Define uncertain variables
zeta = sdpvar(2, 1);     % Uncertain variables zeta1 and zeta2

% Declare zeta as uncertain
uncertain(zeta);

% Define constraints
Constraints = [
    x(1) >= zeta(1),
    x(2) >= zeta(2),
    norm(zeta, 1) <= 1   % Uncertainty set: ||zeta||_1 <= 1
];

% Define objective function
Objective = x(1) + x(2);

% Set solver options
options = sdpsettings('solver', 'mosek', 'verbose', 1);

% Solve the robust optimization problem
sol = optimize(Constraints, Objective, options);

% Check solution status
if sol.problem == 0
    % Extract and display the optimal solution
    robust_optimal_x = value(x);
    fprintf('Robust optimal solution for x:\n');
    disp(robust_optimal_x);
else
    % Display error information
    disp('Solver failed to find an optimal robust solution.');
    disp(sol.info);
    yalmiperror(sol.problem);
end
