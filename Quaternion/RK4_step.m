function [ X, dX ] = RK4_step( f, X, t, dt )
%RK4 is one time step of a runge kutta 4th order step

% approximate derivative of function f
k1 = f( t, X );
k2 = f( t + dt/2 , X + ((dt/2)*k1) );
k3 = f( t + (dt/2) , X + ((dt/2)*k2) );
k4 = f( t + dt , X + k3*dt );

dX =(k1 + 2*k2 + 2*k3 + k4)/6;

% numerical solution for next step of f
X = X + ( dt * dX );



end

