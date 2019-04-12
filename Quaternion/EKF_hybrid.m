function [Xhat, P] = EKF_hybrid(t, Xhat ,u , y ,P, Q, R, g, m, In, Cf,...
                            Cm, Km, Le, dt)
% hybrid EKF
Vw = [0;0;0];
N = length(Xhat);
L = eye(N);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% hybrd part: integration of  continuous dynamics %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% RK4 for X_hat Priori and P Priori %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = ModelJacobian( Xhat, u, g, Le, In, m, Km, Cf, Cm );
dP1 = A*P + P*A' + L*Q*L';
dX1 = QuadCopterNLPlant_Quaternion( t, Xhat, u, g, m, In, Cf, Cm, Km, Le, Vw );

Pp = P + (dP1*dt*.5);                                               % P prime is the estimate of P at half a time step ahead
Xhatp = Xhat+(dX1*dt*.5);                                         % X_hat prime is the estimate of X-aht at half a time step ahead
A = ModelJacobian( Xhatp, u, g, Le, In, m, Km, Cf, Cm );
dP2 = A*Pp + Pp*A' + L*Q*L';
dX2 = QuadCopterNLPlant_Quaternion( t, Xhatp, u, g, m, In, Cf, Cm, Km, Le, Vw );

Pp = P + (dP2*dt*.5);                                               % P prime is the estimate of P at half a time step ahead
Xhatp = Xhat+(dX2*dt*.5);                                         % X_hat prime is the estimate of X-aht at half a time step ahead
A = ModelJacobian( Xhatp, u, g, Le, In, m, Km, Cf, Cm );
dP3 = A*Pp + Pp*A' + L*Q*L';
dX3 = QuadCopterNLPlant_Quaternion( t, Xhatp, u, g, m, In, Cf, Cm, Km, Le, Vw );

Pp = P + (dP3*dt);                                               % P prime is the estimate of P at a time step ahead
Xhatp = Xhat+(dX3*dt);                                        % X_hat prime is the estimate of X-aht ata time step ahead
A = ModelJacobian( Xhatp, u, g, Le, In, m, Km, Cf, Cm );
dP4 = A*Pp + Pp*A' + L*Q*L';
dX4 = QuadCopterNLPlant_Quaternion( t, Xhatp, u, g, m, In, Cf, Cm, Km, Le, Vw );

dP_priori = ( dP1 +  2*dP2 +  2*dP3 + dP4 ) / 6;
dXhat_priori = ( dX1 + 2*dX2 + 2*dX3 + dX4 ) / 6;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% hybrd part: integration of  continuous dynamics %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% rectangular rule for X_hat Priori and P Priori %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A = ModelJacobian( Xhat, u, g, Le, In, m, Km, Cf, Cm );
% dP_priori = A*P + P*A' + L*Q*L';
% dXhat_priori = QuadCopterNLPlant_Quaternion( t, Xhat, u, g, m, In, Cf, Cm, Km, Le, Vw );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P_priori = P + dt *  dP_priori;             %
Xhat_priori = Xhat + dt *  dXhat_priori; %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% the linearized measurments H matrix
C = MeasurmentJacobian( y, Xhat_priori, m, Cf, g );
% Kalman Gain
M = eye(length(y));
K = (P_priori*C')/(C*P_priori*C' + M*R*M');           % NxM gain matrix K_EKF = P_priori*C'*inv(C*P_priori*C' + M*R*M')
h = MeasurmentDynamics(Xhat_priori, g, m, Cf );
% K = ones(17,6);
% correct
Xhat = Xhat_priori + K*(y - h);
P = ( eye(N) - K*C )*P_priori;




end