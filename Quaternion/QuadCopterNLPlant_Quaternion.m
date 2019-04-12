function dX = QuadCopterNLPlant_Quaternion( t, X, u, g, m, I, Cf, Cm, Km, L, Vw ) %#ok<INUSL>
%QUADCOPTERPLANT_QUATERNION
%%%%notes
% _b indicates Body Frame and _f indicates fixed frame suffix d indicates 
% dot (time derivative)
% general dynamics referenced from warren f phillips mechanics of flight 
% pg .753
% quadcopter dyamics referenced from "the GRASP Multiple micro-UAV test
% bed"
% state-vector: X = [u v w p q r xf yf zf e0 ex ey ez w1 w2 w3 w4]'
% the input u is desired angular velocity changes for the motors to be 
% determined by the attitude controller and wind speed disturbance
% Control -Input: u = [w1_d w2_d w3_d w4_d]
% g = gravity
% m = mass 
% I  = moment of inertia matrix
% Cf = coef of force
% Cm = coef of moment
% Km = motor "time constant"


%%%% Forces and moments produced by motor in body frame
w_sqrd = (X(14:17).^2);
% Forces
F = (Cf * w_sqrd);
Fb = [ 0; 
       0;
      -sum(F) ]; 
% Moments
M = Cm * w_sqrd;
Mb = [ L*(F(2)-F(4)); 
       L*(F(1)-F(3));
       M(1)-M(2)+M(3)-M(4) ];



%%%% Rotation matrices
R_b2f = Rotation_b2f_Q( X(10:13,1) );             % go from body to fixed
R_f2b = R_b2f';                             % go from fixed to body
a2q = Ang_rate2Q_rate( X(10:13,1) );        % matrix to convert from angular rate to quaternion rate

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%% Non-Linear Rigid Body Dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % state-vector: X = [u v w p q r xf yf zf e0 ex ey ez w1 w2 w3 w4]'
% % quadcopter dynamics
dX(1:3,1) = (1/m)*Fb + g*R_f2b(1:3,3) - cross( X(4:6,1), X(1:3,1) );      % acceleration in body frame (force balance)
dX(4:6,1) = I \ ( Mb - cross(X(4:6,1),I*X(4:6,1)) );    % angular acceleration in body frame( moment balance)
dX(7:9,1) =  R_b2f * X(1:3,1) + Vw; % velocity in inertial frame
dX(10:13,1) = a2q*X(4:6,1);   % orientation (quaternion) rates 
% motor dynamics
dX(14) = alpha( X(14), u(1), Km );
dX(15) = alpha( X(15), u(2), Km );
dX(16) = alpha( X(16), u(3), Km );
dX(17) = alpha( X(17), u(4), Km );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end