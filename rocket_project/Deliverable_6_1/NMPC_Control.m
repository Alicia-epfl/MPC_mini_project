function opti_eval = NMPC_Control(rocket, H)

import casadi.*
opti = casadi.Opti(); % Optimization problem

N = ceil(H/rocket.Ts); % MPC horizon
nx = 12; % Number of states
nu = 4;  % Number of inputs
display(N)

% Decision variables (symbolic)
X_sym = opti.variable(nx, N); % state trajectory
U_sym = opti.variable(nu, N-1);   % control trajectory)

% Parameters (symbolic)
x0_sym  = opti.parameter(nx, 1);  % initial state
ref_sym = opti.parameter(4, 1);   % target position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
% Runge-Kutta 4 integration
f_discrete = @(x,u) RK4(x,u,rocket.Ts,rocket);
Xs = [0;0;0;0;0;ref_sym(4);0;0;0;ref_sym(1);ref_sym(2);ref_sym(3)];
display(rocket.Ts);

alpha = X_sym(4,:);
beta = X_sym(5,:);

delta1 = U_sym(1,:);
delta2 = U_sym(2,:);
Pavg = U_sym(3,:);
Pdiff = U_sym(4,:);
% SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
% H = 0.7 30 30 1 1 1 150 40 40 100 500 500 650
% H = 1 1 1 1 1 1 150 20 20 40 90 90 200
% H = 50 50 50 1 1 150 20 20 50 150 150 250
R = eye(nu);
Q = eye(nx);
Q(1,1) = 50; %wx          
Q(2,2) = 50; %wy         
Q(3,3) = 50; %wz         
Q(4,4) = 1;  %alpha     
Q(5,5) = 1;  %beta      
Q(6,6) = 150; %gamma  
Q(7,7) = 20;     % vx;
Q(8,8) = 20;    % vy; 
Q(9,9) = 50;     % vz;
Q(10,10) = 150; %     x
Q(11,11) = 150;  %    y
Q(12,12) = 300; %     z
%display(Q)

% ---- objective ---------
obj = 0;
opti.subject_to(X_sym(:,1)==x0_sym);
[xs, Us] = rocket.trim(); % we don t use litte xs


for k=1:N-1 % loop over control intervals
    opti.subject_to(X_sym(:,k+1) == f_discrete(X_sym(:,k), U_sym(:,k)));
    obj = obj + (X_sym(:,k)-Xs)'*Q*(X_sym(:,k)-Xs) + (U_sym(:,k)-Us)'*R*(U_sym(:,k)-Us); 
end

%opti.subject_to(-0.087 <= alpha <= 0.087); % alpha 
opti.subject_to(-1.48 <=beta <= 1.48); % beta

%display(U_sym);
%display(X_sym);

opti.subject_to( -0.26 <= delta1 <= 0.26);
opti.subject_to(-0.26 <= delta2 <= 0.26);

opti.subject_to(50 <= Pavg <= 80);
opti.subject_to(-20 <= Pdiff <= 20);

opti.minimize(obj);


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
