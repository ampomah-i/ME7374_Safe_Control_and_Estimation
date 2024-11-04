% Define the system matrix A for a = 0.5
A = [0.5 -1; 0 -1];

% Introduce a small perturbation (damping)
epsilon = 0.01;
A_epsilon = A + epsilon * eye(size(A));

% Define the matrix Q as the identity matrix (or another positive definite matrix)
Q = eye(2);

% Solve the discrete-time Lyapunov equation with the perturbed matrix A_epsilon
P = dlyap(A_epsilon', Q);

% Display the resulting matrix P
disp('The matrix P for perturbed A is:');
disp(P);

% Check if P is positive definite by examining its eigenvalues
eig_P = eig(P);
disp('The eigenvalues of P are:');
disp(eig_P);

% Compute A'PA - P
lyap_diff = A_epsilon' * P * A_epsilon - P;

% Display the matrix A'PA - P
disp('The matrix A_epsilon^T P A_epsilon - P is:');
disp(lyap_diff);

% Check the eigenvalues to verify if it is negative semi-definite
eig_lyap_diff = eig(lyap_diff);
disp('The eigenvalues of A_epsilon^T P A_epsilon - P are:');
disp(eig_lyap_diff);

% Check if all eigenvalues are non-positive
if all(eig_lyap_diff <= 0)
    disp('The matrix is negative semi-definite.');
else
    disp('The matrix is not negative semi-definite.');
end