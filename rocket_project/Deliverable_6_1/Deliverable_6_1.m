%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EPFL | Semester: fall 2021                 %
% Alicia Mauroux | sciper: 274618            %
% ME-425: Model predictive control | part 6  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath(fullfile('..', 'src'));

Ts = 1/10; % Note that we choose a larger Ts here to speed up the simulation
rocket = Rocket(Ts);
H = 0.5;
nmpc = NMPC_Control(rocket, H);
% MPC reference with default maximum roll = 15 deg
Tf = 30;
%ref = @(t_, x_) rocket.MPC_ref(t_, Tf);
% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) rocket.MPC_ref(t_, Tf,roll_max);
x0 = zeros(12,1);
%display('simulation')
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, nmpc, ref);



% TODO: This file should produce all the plots for the deliverable
% Plot pose
rocket.anim_rate = 1; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
