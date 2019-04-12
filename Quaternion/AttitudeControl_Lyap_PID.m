function [w, ei, ep, delW_ang_lyap, delta] = AttitudeControl_Lyap_PID( X, u, Kp, Ki, Kd, dt,...
    ep0, ei0, delW_Vertical, Cm, Cf, I, L, w_h )
%ATTITUDECONTROL_PD Summary of this function goes here
% referenced from "the GRASP Multiple micro-UAV test bed"
% state-vector: X = [u v w p q r xf yf zf e0 ex ey ez w1 w2 w3 w4]'
% input u = [phi_des theta_des psi_des p_des q_des r_des]
% delW_Vertical = delW_F + w_h


eAngs = Q2eAng( X(10:13,1) );

%%%%%%%% error from desired setpoint
ep = u(1:3,1) - eAngs;             % error in desired attitude
% ed = u(4:6,1) - X(4:6,1);          % error in desired attitude derivative
ed = (ep - ep0)/dt;          % error in desired attitude derivative
ei = ei0 + (dt/2)*( ep0 + ep );    % error in desired attitude integral (using Trap Rule)


%%%%%%%%% PID control Laws %%%%%%%%%%%%%%%
delW_ang_PID = Kp.*ep + Kd.*ed + Ki.*ei;

%%%%%%% Lyapunov Control Laws %%%%%%%%%%%%
% calculate size of uncertainty

a = 4*L*Cf*w_h/I(1,1);
b = 4*L*Cf*w_h/I(2,2);
c = 8*Cm*w_h/I(3,3);

% first calculate Forces Moments
w_sqrd = X(14:17).^2;
% Forces
F = Cf * w_sqrd;
% moments
M = Cm * w_sqrd;
Mb = [ L*(F(2)-F(4)); 
       L*(F(1)-F(3));
       M(1)-M(2)+M(3)-M(4) ];
NL_pqr_dot = I \ ( Mb - cross(X(4:6,1),I*X(4:6,1)) ); % non linear model

w2wDes = [0.25  0.25  0.25  0.25;
          0.00  0.50  0.00 -0.50;
          0.50  0.00 -0.50  0.00;
          0.25 -0.25  0.25 -0.25];
delW0 = w2wDes*X(14:17);
A = [a 0 0;0 b 0; 0 0 c];

Lin_pqr_dot = A * delW0(2:4);       % linear model

delta = NL_pqr_dot - Lin_pqr_dot; % uncertainty!!!
delta = delta * 3;                % increase it to increase work of NL control

P = [0.261394759087066,0,0,-0.499999999999999,0,0,-1.88165680473373,0,0;0,0.261394759087066,0,0,-0.499999999999999,0,0,-1.88165680473373,0;0,0,0.261394759087066,0,0,-0.499999999999999,0,0,-1.88165680473373;-0.499999999999999,0,0,1.88165680473373,0,0,-0.500000000000012,0,0;0,-0.499999999999999,0,0,1.88165680473373,0,0,-0.500000000000012,0;0,0,-0.499999999999999,0,0,1.88165680473373,0,0,-0.500000000000012;-1.88165680473373,0,0,-0.500000000000012,0,0,42.2988165680474,0,0;0,-1.88165680473373,0,0,-0.500000000000012,0,0,42.2988165680474,0;0,0,-1.88165680473373,0,0,-0.500000000000012,0,0,42.2988165680474];

omega_lyap = [2*a*( X(4,1)*P(3,3) + X(5,1)*P(3,6) + X(6,1)*P(3,9) );
              2*b*( X(4,1)*P(6,3) + X(5,1)*P(6,6) + X(6,1)*P(6,9) );
              2*c*( X(4,1)*P(9,3) + X(5,1)*P(9,6) + X(6,1)*P(9,9) )];

delW_ang_lyap = abs(delta).*sign(omega_lyap);

delta = delta/3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% calculate desired rotor speeds %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% delW_ang = delW_ang_PID - delW_ang_lyap;
delW_ang = delW_ang_PID; % just PID control
delW = [delW_Vertical;delW_ang];

wDes2w = [1  0  1  1;
          1  1  0 -1;
          1  0 -1  1;
          1 -1  0 -1];
         
w = wDes2w*delW ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end

