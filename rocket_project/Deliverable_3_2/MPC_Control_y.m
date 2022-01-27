classdef MPC_Control_y < MPC_Control
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N = ceil(H/Ts); % Horizon steps

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            R = 20*eye(nu);
            Q = 1*eye(nx);
            Q(2,2)= 10;
            Q(4,4)= 0.5;
            
            [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
            K=-K;
            Acl= mpc.A+mpc.B*K;
            
            % Constraints
            M = [1;-1]; m = [deg2rad(15); deg2rad(15)];
            F = [0 1 0 0; 0 -1 0 0]; f = [deg2rad(5); deg2rad(5)];
            
            % Terminal set
            Xf = polytope([F;M*K],[f;m]);
            
            while 1
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf,preXf);
                if isequal(prevXf, Xf)
                    break
                end
            end
            [Ff,ff]=double(Xf);

            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE                
            con = [];
            obj = 0;
            
            con = (X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1)) + (M*U(:,1) <= m) + (F*X(:,1) <= f);
            obj = (X(:,1)-x_ref)'*Q*(X(:,1)-x_ref) + (U(:,1)-u_ref)'*R*(U(:,1)-u_ref);
            for i = 2:N-1
                con = con + (X(:,i+1) == mpc.A*X(:,i) + mpc.B*U(:,i));
                con = con + (F*X(:,i) <= f) + (M*U(:,i) <= m);
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref) + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref);
            end
            con = con + (Ff*(X(:,N)-x_ref)<= ff);
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref);
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, U(:,1));
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            Rs = 1;
            con = [mpc.A*xs + mpc.B*us == xs; mpc.C*xs == ref;
                     -deg2rad(15) <= us <= deg2rad(15); -deg2rad(5)<=xs(2)<=deg2rad(5)];
            obj = us*Rs*us;
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
