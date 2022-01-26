function [x_next] = RK4(X,U,H,rocket)
%
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future
%
display('RK4')
%
%
% Runge-Kutta 4 integration
%

   k1 = rocket.f(X,U);
   k2 = rocket.f(X+H/2*k1, U);
   k3 = rocket.f(X+H/2*k2, U);
   k4 = rocket.f(X+H*k3,   U);
   x_next = X + H/6*(k1+2*k2+2*k3+k4);


end