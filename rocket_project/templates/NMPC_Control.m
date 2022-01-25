function opti_eval = NMPC_Control(rocket, H)

import casadi.*
opti = casadi.Opti(); % Optimization problem

N = ceil(H/rocket.Ts); % MPC horizon
nx = 12; % Number of states
nu = 4;  % Number of inputs

% Decision variables (symbolic)
X_sym = opti.variable(nx, N); % state trajectory
U_sym = opti.variable(nu, N-1);   % control trajectory)

% Parameters (symbolic)
x0_sym  = opti.parameter(nx, 1);  % initial state
ref_sym = opti.parameter(4, 1);   % target position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

% Runge-Kutta 4 integration
k1 = rocket.f(X_sym,        U_sym);
k2 = rocket.f(X_sym+h/2*k1, U_sym);
k3 = rocket.f(X_sym+h/2*k2, U_sym);
k4 = rocket.f(X_sym+h*k3,   U_sym);
opti.subject_to(X_sym(:,k+1) == X_sym + h/6*(k1+2*k2+2*k3+k4));

% SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            
R = 0.5*eye(nu);
Q = 10*eye(nx);

% ---- objective ---------
opti.minimize(...
    sum(X_sym(2,:)'*Q*X_sym(2,:))  + ... % min x'Qx
  U_sym(1,:)'*R*U_sym(1,:) );             % min u'Ru
  

% ---- constraints -----------
M = [1;-1]; m = [deg2rad(15); deg2rad(15)];
F = [0 1 0 0; 0 -1 0 0]; f = [deg2rad(85); deg2rad(85)];

opti.subject_to(20 <= U(4) <= 90);  % Pdiff is limited
opti.subject_to(50 <= U(3) <= 90);  % Pavg is limited


opti.subject_to(M*U_sym <= m);   % deta contraint not sure
opti.subject_to(F*X_sym <= f);   % beta contraint


% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ---- Setup solver ------
ops = struct('ipopt', struct('print_level', 0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);

% Create function to solve and evaluate opti
opti_eval = @(x0_, ref_) solve(x0_, ref_, opti, x0_sym, ref_sym, U_sym);
end

function u = solve(x0, ref, opti, x0_sym, ref_sym, U_sym)

% ---- Set the initial state and reference ----
opti.set_value(x0_sym, x0);
opti.set_value(ref_sym, ref);

% ---- Solve the optimization problem ----
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');

u = opti.value(U_sym(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end
