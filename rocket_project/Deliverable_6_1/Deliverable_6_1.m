%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EPFL | Semester: fall 2021                 %
% Alicia Mauroux | sciper: 274618            %
% ME-425: Model predictive control | part 6  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath(fullfile('..', 'src'));

Ts = 1/10; % Note that we choose a larger Ts here to speed up the simulation
rocket = Rocket(Ts);
[xs, us] = rocket.trim() % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us) % Linearize the nonlinear model about trim point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us)

%H = 4;
%nmpc = NMPC_Control(rocket, H);


%% Design MPC controller
H = 3; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
%%
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);
%% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% MPC reference with default maximum roll = 15 deg
Tf = 30;
%ref = @(t_, x_) rocket.MPC_ref(t_, Tf);
%% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) rocket.MPC_ref(t_, Tf,roll_max);
x0 = zeros(12,1);
%display('simulation')
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);

%% TODO: This file should produce all the plots for the deliverable
% Plot pose
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
