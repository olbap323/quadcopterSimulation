function [u_att, delW_F, ei, ep]...
    = HoverControl( X, u, dt, Kp, Ki, Kd, m, g, ep0, ei0, Cf, w_h)
%HOVERCONTROL_PID Summary of this function goes here
% referenced from "the GRASP Multiple micro-UAV test bed"
% state-vector: X = [u v w p q r xf yf zf e0 ex ey ez w1 w2 w3 w4]'
% input u = [rx_T ry_T rz_T rxd_T ryd_T rzd_T psi0]


r_T = u(1:3,1);     % Desired Trajectory vector {x,y,z} in world frame
rd_T = u(4:6,1);    % Desired Velcoity vector {xd,yd,zd} in world frame

r = X(7:9,1);     % Trajectory vector {x,y,z} in world frame
rd = X(1:3,1);    % Velcoity vector {xd,yd,zd} in world frame
R = Rotation_b2f_Q( X(10:13,1) );
rd = R*rd;

ep = (r_T - r);            % error of trajectory
ed = (rd_T - rd);         % error of velocity
ei = ei0 + (dt/2)*( ep0 + ep ); % error of trajectory integral

rdd_des = Kd.*ed + Kp.*ep + Ki.*ei;

phi_des = (1/g) * ( rdd_des(1)*sin(u(7)) - rdd_des(2)*cos(u(7)) );
theta_des = (1/g) * ( rdd_des(1)*cos(u(7)) + rdd_des(2)*sin(u(7)) );
psi_des = u(7);
delW_F = rdd_des(3)*m/8/Cf/w_h;
% delW_F = rdd_des(3)*m/8/Cf;
p_des = 0;
q_des = 0;
r_des = 0;

u_att = [phi_des; theta_des; psi_des; p_des; q_des; r_des];

end


