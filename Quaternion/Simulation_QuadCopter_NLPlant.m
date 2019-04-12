%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% QuadCopter Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% - Non Linear rigid body dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% - PID w/ Lyapunov Redesign Atitude Control Law %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% * ( linear Model assumed ) %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% - PID Position (Hover/3D-Trajectory) Control Law %%%%%%%%%% 
%%%%%%%%%%%%%%%%%%% * ( linear Model assumed ) %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
labelFont = 12;
fN = 1;
rads2rpm = 60/2/pi;
r2d = 180/pi;
gridsOn = true;
% d2r = pi/180;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% prepare model constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coniguration constants
g = 9.81;                               % accelration of gravity [m/s/s]
m = .7065;                              % quad-copter mass [kg]
L = .225;                               % quad-copter moment arm [m] 
In = diag( [ 0.0062 0.0062 0.0121 ] );  % quad-copter moment of inertia [kg * m^2]
% dynamics constants
Cf = 7.5509e-06;                        % coef of force [kg*m]
Cm = 1e-7;                             % coef of moment [kg*m]
Km = 10;                                % linear motor acceleration rate [1/s]
% Cf = 6.11e-08;                        % coef of force [kg*m]
% Cm = 1.5e-9;                             % coef of moment [kg*m]
% Km = 20;                                % linear motor acceleration rate [1/s]
w_h = sqrt(m*g/4/Cf);                   % angular velocity required by each rotor to sustain level hover flight
% control constants
% attitude (PID)
Kp_a = [53; 53; 869];
Ki_a = [72; 72; 1185];
Kd_a = [16; 16; 253];
% Kp_a = Kp;
% Ki_a = Ki;
% Kd_a = Kd;

% hover (PID)
Kp_h = [   -0.75;    -0.75;   -3]*1.0;
Ki_h = [-0.75; -0.75; -3]*1.0;
Kd_h = [   -3;    -3;   -5]*1.0;


% initial state vector
% state-vector: X = [u v w p q r xf yf zf e0 ex ey ez w1 w2 w3 w4]'
% no initial disturbance
X0(7:9,1) = [0 0 -10]';             % [ x_f y_f z_f] [m]
eAngs0 = [0 pi/2.2 0]';
quaternions = eAng2Q( eAngs0 );
X0(10:13,1) = quaternions;               % [phi theta psi] [rad]


X0(1:3,1) = [0 0 0]';               % [u v w] [m/s]
X0(4:6,1) = [0 0 0]';                 % [p q r] [rad/s]

X0(14:17,1) = [w_h w_h w_h w_h]';     % [w1 w2 w3 w4] [rad/s]
% controller sample rates ( simulation runs at fs_att)
fs_att = 400;                       % attitude controller sample rate [Hz]
fs_pos = 100;                       % position controller sample rate [Hz]
fs_obs = 100;                       % state observer estimation sample rate [Hz]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% prepare simulation constants %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t0 = 0;                         % starting time of simulation [s]
tf = 20;                        % final time of simulation [s]
pos_ind = fs_att/fs_pos;    % number of atticitude control updates to perform before position control update
obs_ind = fs_att/fs_obs;        % number of simulation updates updates to perform before observer estimation update
dt = 1/fs_att;                  % time step [s] i.e. the fastest sample rate being simulated (attiude controller) 
dt_PosCont = 1/fs_pos ;
dt_obs = 1/fs_obs;
t = t0:dt:tf;                   % time vector[s]
N = length(t);                  % number of RK4/plant iterations
Np = (N-1)/pos_ind;        % number of position control iterations
No = (N-1)/obs_ind;
tp = linspace(t0,tf,Np);
j=0;                            % inner loop index counter (position controller
k = 0;                          % inner loop index counter (observer estimation)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% prepare observer constants (ekf) %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
run('generateZeroMeanWhiteNoise');

% P01  = accVar;
% P02  = accVar;
% P03  = accVar;
% P04  = gyroVar;
% P05  = gyroVar;
% P06  = gyroVar;
% P07  = 0;
% P08  = 0;
% P09  = 0;
% P010 = 0;
% P011 = 0;
% P012 = 0;
% P013 = 0;
% P014 = 0;
% P015 = 0;
% P016 = 0;
% P017 = 0;
% P0 = diag([P01 P02 P03 P04 P05 P06 P07 P08 P09 P010 P011 P012 P013 P014 P015 P016 P017]);
% clearvars P01 P02 P03 P04 P05 P06 P07 P08 P09 P010 P011 P012 P013 P014 P015 P016 P017;
P0 = eye(17);

Q1  = 1e0;
Q2  = 1e0;
Q3  = 1e0;
Q4  = 1e0;
Q5  = 1e0;
Q6  = 1e0;
Q7  = 1e0;
Q8  = 1e0;
Q9  = 1e0;
Q10 = 1e0;
Q11 = 1e0;
Q12 = 1e0;
Q13 = 1e0;
Q14 = 1e0;
Q15 = 1e0;
Q16 = 1e0;
Q17 = 1e0;
Q = diag([Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8 Q9 Q10 Q11 Q12 Q13 Q14 Q15 Q16 Q17]*0);
Q = Q/dt_obs;
clearvars Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8 Q9 Q10 Q11 Q12 Q13 Q14 Q15 Q16 Q17;

R1  = accVar^2;
R2  = accVar^2;
R3  = accVar^2;
R4  = gyroVar^2;
R5  = gyroVar^2;
R6  = gyroVar^2;
R= diag([R1 R2 R3 R4 R5 R6]*1);
% R = R/dt_obs;
clearvars R1 R2 R3 R4 R5 R6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% decalre space/ control inputs/ model pertubations %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% declare memory

X = ones(17,N);                 % declare memory for state vector 
dX = ones(17,N);                 % declare memory for state vector derivatve
X(:,1) = X0;                    % place the initial condition
Xhat = ones(17,No);                 % declare memory for state estimate vector 
Xhat(:,1) = X0;                    % place the initial condition  
P = zeros(17,17,No);            % declare memory for covariance matrix
P(:,:,1) = P0;
Pt = zeros(1,No);               % decalre memroy for the trace of the covariance matrix
u_pos = ones(7,N);              % declare memory for position control input
u_plant = zeros(4,N);           % declare memory for plant input vector to be filled by simulation
u_att = zeros(6,Np);             % declare memory for attitude control input vector to be filled by simulation
Vw = ones(3,N);                 % declare memory for wind perturbation
hovTs = zeros(1,N);             % declare memeroy for hover controller times
attTs = zeros(1,N);             % declare memeroy for attitude controller times
plntTs = zeros(1,Np);          % declare memeroy for plant times
EKF_Ts = zeros(1,Np);          % declare memeroy for EKF times
ei_p = zeros(3,Np);            % declare memory for time history integral of position error
ei_a = zeros(3,N);              % declare memory for time history  integral of attitude error
ep0_p = zeros(3,1);             % holds the old proportional error for position controller
ep0_a = zeros(3,1);             % holds the old proportional error for position controller
omega_lyap = zeros(3,N);        % The lyapunov constant that has the sign 
delta = zeros(3,N);


% set the desired trajectory and velocities 
%%%%%%%%%%%%%%%%%%%%% HOVER CONTROLLER INPUTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u_pos(1,:) = u_pos(1,:) * 0;      % rx_T [m]
u_pos(2,:) = u_pos(2,:) * 0;      % ry_T [m]
u_pos(3,:) = u_pos(3,:) * -10;    % rz_T [m]
u_pos(4,:) = u_pos(4,:) * 0;      % rxd_T [m/s]
u_pos(5,:) = u_pos(5,:) * 0;      % ryd_T [m/s]
u_pos(6,:) = u_pos(6,:) * 0;      % rzd_T [m/s]
u_pos(7,:) = u_pos(6,:) * eAngs0(3); % psi_T [m/s]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% model Perturbation
% ps = N/2;               % pertubation step
% Vw(1,1:ps) = Vw(1,1:ps)*0;
% Vw(1,ps+1:end) = Vw(1,ps+1:end)*2;
Vw(1,:) = Vw(1,:)*0;
Vw(2,:) = Vw(2,:)*0;
Vw(3,:) = Vw(3,:)*0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:N-1,

    if t(i) > 3.0,
        meow = 1;
    end
    %%%%%%%%%%%%% position control update %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if mod( i-1, pos_ind) == 0,
        j = j + 1;
        tic;
        [u_att(:,j), delW_F, ei_p(:,j+1), ep0_p  ] =...
                HoverControl( X(:,i), u_pos(:,i), dt_PosCont, Kp_h, Ki_h,...
                            Kd_h, m, g, ep0_p, ei_p(:,j), Cf, w_h);
        
        delW_Vertical = w_h + delW_F;
        hovTs(i) = toc;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    
    %%%%%%%%%%%%% attitude control update %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tic;
    [u_plant(:,i), ei_a(:,i), ep0_a, omega_lyap(:,i), delta(:,i) ]=....
        AttitudeControl_Lyap_PID( X(:,i),u_att(:,j), Kp_a, Ki_a, Kd_a, dt,...
        ep0_a, ei_a(:,i), delW_Vertical, Cm, Cf, In, L, w_h ); % attitude control 
     
    attTs(i) = toc;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

    %%%%%%%%%%%%% rigid-body dynamics update %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tic;
    [X(:,i+1), dX(:,i+1)] = RK4_step(  ...
        @(t,X)QuadCopterNLPlant_Quaternion( t, X, u_plant(:,i), g, m,...
        In, Cf, Cm, Km, L, Vw(:,i) ), X(:,i), t(i), dt );
    plntTs(i) = toc;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%% state estimator update %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if mod( i-1, obs_ind) == 0,
        k = k + 1;
        y = [ dX(1:3,i+1); X(4:6,i+1)] + whiteNoise(:,k);
        tic;
        [Xhat(:,k+1), P(:,:,k+1)] = EKF_hybrid(t(i) , Xhat(:,k) ,u_plant(:,i) ,...
            y ,P(:,:,k), Q, R, g, m, In, Cf, Cm, Km, L, dt_obs);
        EKF_Ts(i) = toc;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% all the good data neatly in struct for ease of plotting 
data.X = X;
data.u = u_plant;
data.t = t;

run('PlotSimulation.m');
% clearvars -except data X t u_plant
