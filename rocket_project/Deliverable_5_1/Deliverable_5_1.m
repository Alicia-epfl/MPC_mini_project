addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);


%% Design MPC controller
H = 3; % Horizon length in seconds %do we need it?
mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

%% Simulation
% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
% Setup reference function
Tf = 30;
ref = @(t_, x_) rocket.MPC_ref(t_, Tf);
x0 = zeros(12,1);

%Test_offset = true; %Indicate if we are testing 5.1 or not
Test_offset = true; 
rocket.mass = 1.783; % Manipulate mass for simulation

if Test_offset
    %In order to test our modification with the mass: the controller
    %will now act based on the state estimates from the observer. You can obtain the estimates of the
    %z states from the corresponding columns of the Z hat output. The last row is the disturbance
    %estimate d.
    [T, X, U, Ref, Z_hat] = rocket.simulate_f_est_z(x0, Tf, mpc, ref, mpc_z, sys_z);
    Zhatsize = size(Z_hat)
    
    % Plot pose
    rocket.anim_rate = 10; % Increase this to make the animation faster
    ph = rocket.plotvis(T, X, U, Ref);
    ph.fig.Name = 'Merged lin. MPC offset free in nonlinear simulation'; % Set a figure title
else
   [T, X, U, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);
 
    % Plot pose
    rocket.anim_rate = 10; % Increase this to make the animation faster
    ph = rocket.plotvis(T, X, U, Ref);
    ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
end








