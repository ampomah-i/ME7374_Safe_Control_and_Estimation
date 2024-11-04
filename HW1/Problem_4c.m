% Parameters
r1 = 1; 
r2 = 1; 
v1 = 1; 
v2 = 1.8; 
w = 2.5;

% Initial conditions
x1_0 = 2; 
x2_0 = 2.5;
q0 = 1;  

% Initial state vector
z0 = [x1_0; x2_0; q0];

% Time horizon for simulation
TSPAN = [0 15];  % 15 seconds

% Jump conditions
JSPAN = [0 100]; 

% Set solver tolerances (adjust as necessary)
options = odeset('RelTol',1e-6,'MaxStep',0.01);

% Set the rule for the solver (1 for jumps priority, 2 for flows priority)
rule = 1;

% Run the hybrid system simulation
[t, j, z] = HyEQsolver(@f, @g, @C, @D, z0, TSPAN, JSPAN, rule, options);

% Plot results
figure;
subplot(3,1,1);
plot(t, z(:,1), 'LineWidth', 2); % x1 (Tank 1 level)
xlabel('Time [s]');
ylabel('x1 (Tank 1 level)');
grid on;

subplot(3,1,2);
plot(t, z(:,2), 'LineWidth', 2); % x2 (Tank 2 level)
xlabel('Time [s]');
ylabel('x2 (Tank 2 level)');
grid on;

subplot(3,1,3);
stairs(t, z(:,3), 'LineWidth', 2); % q (Discrete state)
xlabel('Time [s]');
ylabel('q (Discrete state)');
grid on;

% Function Definitions

% Define the flow map (continuous dynamics)
function dz = f(z)
    % Parameters
    v1 = 1; 
    v2 = 1.8; 
    w = 2.5;
    
    x1 = z(1);
    x2 = z(2);
    q = z(3);  % Discrete state
    
    % Ensure q is an integer
    q = round(q);
    
    % Define the flow dynamics based on discrete state q
    if q == 1  % q = q1, inflow to Tank 1
        dx1 = w - v1;
        dx2 = -v2;
        dq = 0;  % No change in discrete state
    elseif q == 2  % q = q2, inflow to Tank 2
        dx1 = -v1;
        dx2 = w - v2;
        dq = 0;  % No change in discrete state
    else
        error('Unexpected value of q: %f', q);
    end
    
    % Return the continuous dynamics
    dz = [dx1; dx2; dq];
end

% Define the jump map (discrete transitions)
function zplus = g(z)
    x1 = z(1);
    x2 = z(2);
    q = z(3);
    
    % Parameters
    r1 = 1; 
    r2 = 1;
    
    % Switch between q1 and q2 based on thresholds
    if q == 1 && x2 <= r2  % Switch from q1 to q2
        zplus = [x1; x2; 2];  % Switch to q2
    elseif q == 2 && x1 <= r1  % Switch from q2 to q1
        zplus = [x1; x2; 1];  % Switch to q1
    else
        zplus = z;  % No switch
    end
end

% Define the flow set (when continuous evolution occurs)
function inside = C(z)
    x1 = z(1);
    x2 = z(2);
    q = z(3);
    
    % Parameters
    r1 = 1; 
    r2 = 1;
    
    % Flow occurs when not at the threshold
    inside = (q == 1 && x2 > r2) || (q == 2 && x1 > r1);
    inside = double(inside);  % Convert logical to double (1 or 0)
end

% Define the jump set (when discrete transitions occur)
function inside = D(z)
    x1 = z(1);
    x2 = z(2);
    q = z(3);
    
    % Parameters
    r1 = 1; 
    r2 = 1;
    
    % Jump occurs at or below the threshold
    inside = (q == 1 && x2 <= r2) || (q == 2 && x1 <= r1);
    inside = double(inside);  % Convert logical to double (1 or 0)
end
