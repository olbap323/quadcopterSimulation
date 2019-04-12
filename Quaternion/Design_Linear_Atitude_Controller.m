clc
clear all;

g = 9.81;                               % accelration of gravity [m/s/s]
m = .7065;                              % quad-copter mass [kg]
L = .225;                               % quad-copter moment arm [m] 
In = diag( [ 0.0062 0.0062 0.0121 ] );   % quad-copter moment of inertia [kg * m^2]
% dynamics constants
Cf = 7.5509e-06;                        % coef of force [kg*m]
Cm = 1e-7;                             % coef of moment [kg*m]
w_h = sqrt(m*g/4/Cf); 


a = 4*L*Cf*w_h/In(1,1);
b = 4*L*Cf*w_h/In(2,2);
c = 8*Cm*w_h/In(3,3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Linear system with PID law %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% linear models

phi.A = [0 1 0;
         0 0 1;
         0 0 0];
phi.B = [0;
         0;
         a];

theta.A = [0 1 0;
           0 0 1;
           0 0 0];
theta.B = [0;
           0;
           b];

psi.A = [0 1 0;
         0 0 1;
         0 0 0];
psi.B = [0;
         0;
         c];

hoverz.A = [0 1 0;
            0 0 1;
            0 0 0];
hoverz.B = [0;
            0;
            -4*Cf/m];  
% pole placements
% phi.p = [-10; -5 + 5*1i;-5 - 5*1i];
% theta.p = [-10; -5 + 5*1i;-5 - 5*1i];
% psi.p = [-10; -5 + 5*1i;-5 - 5*1i];

% phi.p = [-1; -1 + 1*1i;-1 - 1*1i]*2;
% theta.p = [-1; -1 + 1*1i;-1 - 1*1i]*2;
% psi.p = [-1; -1 + 1*1i;-1 - 1*1i]*2;

phi.p = [-3; -2.5 + 2.5*1i;-2.5 - 2.5*1i];
theta.p = [-3; -2.5 + 2.5*1i;-2.5 - 2.5*1i];
psi.p = [-3; -2.5 + 2.5*1i;-2.5 - 2.5*1i];

hoverz.p = [-.05; -0.05 + 0.25*1i;-0.05 - 0.25*1i];

% Let MATLAB find neccessary gains
phi.K = place(phi.A,phi.B,phi.p)      
theta.K = place(theta.A,theta.B,theta.p)       
psi.K = place(psi.A,psi.B,psi.p)
hoverz.K = place(hoverz.A,hoverz.B,hoverz.p)
hzk = hoverz.K;
% select gains
Ki = [phi.K(1); theta.K(1); psi.K(1)];
Kp = [phi.K(2); theta.K(2); psi.K(2)];
Kd = [phi.K(3); theta.K(3); psi.K(3)];
% the entire linear system with PID control on each angle is then
PID = [      0       0        0       1       0       0       0       0      0;
             0       0        0       0       1       0       0       0      0;
             0       0        0       0       0       1       0       0      0;
             0       0        0       0       0       0       1       0      0;
             0       0        0       0       0       0       0       1      0;
             0       0        0       0       0       0       0       0      1;
       -a*Ki(1)      0        0 -a*Kp(1)      0       0 -a*Kd(1)      0      0;
            0  -b*Ki(2)       0       0 -b*Kp(2)      0       0 -b*Kd(2)     0;
            0        0  -c*Ki(3)      0       0 -c*Kp(3)      0       0 -c*Kd(3)];
% rts = eig(PID) % uncomment to ensure roots are in the right place

%%%%%%% find lyapunov equation for linear system
P = lyap(PID,eye(9)); % lyapunov matrix
% syms ph_i ph ph_d th_i th th_d ps_i ps ps_d  
% X = [ph_i; ph; ph_d; th_i; th; th_d; ps_i; ps; ps_d]
% Xt = [ph_i ph ph_d th_i th th_d ps_i ps ps_d]
% lyapEq = Xt*P*X
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Linear system with state feedback law %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% F = [0 0 0 1 0 0 0 0 0;
%      0 0 0 0 1 0 0 0 0;
%      0 0 0 0 0 1 0 0 0
%      0 0 0 0 0 0 1 0 0;
%      0 0 0 0 0 0 0 1 0;
%      0 0 0 0 0 0 0 0 1;
%      0 0 0 0 0 0 0 0 0;
%      0 0 0 0 0 0 0 0 0;
%      0 0 0 0 0 0 0 0 0];
%  G = [0 0 0;
%       0 0 0;
%       0 0 0;
%       0 0 0;
%       0 0 0;
%       0 0 0;
%       a 0 0;
%       0 b 0;
%       0 0 c];
%   
%   
% p = [-7; -5 + 5*1i;-5 - 5*1i;-7; -5 + 5*1i;-5 - 5*1i;-7; -5 + 5*1i;-5 - 5*1i];
% K = place(F,G,p)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%