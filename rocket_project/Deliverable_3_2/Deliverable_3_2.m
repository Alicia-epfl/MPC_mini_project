addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim() % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us) % Linearize the nonlinear model about trim point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us)

%% Design MPC controller
H = 7; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

%% Simulate
Tf=10.0;
[T, X_sub, U_sub] = rocket.simulate(sys_x, [0,0,0,0], Tf, @mpc_x.get_u, -5);
ph_x=rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, -5);
[T, X_sub, U_sub] = rocket.simulate(sys_y, [0,0,0,0], Tf, @mpc_y.get_u, -5);
ph_y=rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, -5);
[T, X_sub, U_sub] = rocket.simulate(sys_z, [0,0], Tf, @mpc_z.get_u, -5);
ph_z=rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us,-5);
[T, X_sub, U_sub] = rocket.simulate(sys_roll, [0,0], Tf, @mpc_roll.get_u, deg2rad(45));
ph_roll=rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, deg2rad(45));
