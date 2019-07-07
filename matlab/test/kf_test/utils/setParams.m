
%% ======= Van der pol ========
% dt = 0.005;
% tf = 5;
% x0 = [2;0];
% x0_hat = [4.0; 0.8]; % xhat[k|k-1]
% n_dim = length(x0);
% n_msr = 1;
% model = VDP(dt);
%% ======= Kalman filter params ========
% Q = 0.01*eye(n_dim,n_dim); % Variance of the process noise w[k]
% R = 0.25*eye(n_msr,n_msr); % Variance of the measurement noise v[k]
% R_hat = 10*R;
% P0 = 10*eye(n_dim,n_dim);
% use_analytic_jacob = false;


%% ======= Quaternion ========
% dt = 0.005;
% tf = 5;
% Q0 = [0.7 -0.2 -0.6 0.1]';
% Q0 = Q0 / norm(Q0);
% Qg = quatProd( rotm2quat(rotx(50)*rotz(110))', Q0);
% vRot0 = [0; 0; 0];
% x0 = [Q0; vRot0];
% x0_hat = x0;
% n_dim = length(x0);
% n_msr = 7;
% model = Quat5thPolyTrajTracker(Q0, Qg, tf, dt);
%% ======= Kalman filter params ========
% Q = 0.01*eye(n_dim,n_dim); % Variance of the process noise w[k]
% R = 0.25*eye(n_msr,n_msr); % Variance of the measurement noise v[k]
% R_hat = 10*R;
% P0 = 10*eye(n_dim,n_dim);
% use_analytic_jacob = false;


%% ======= Mackey-Glass time series ========
% dt = 0.04;
% tf = 5;
% x0 = [1.2000; 0.9346; 0.7279; 0.5669; 0.4415; 0.3438; 0.2678];
% x0_hat = zeros(size(x0));
% n_dim = length(x0);
% n_msr = 1;
% model = MackeyGlassTS(dt);
%% ======= Kalman filter params ========
% Q = 0.01*eye(n_dim,n_dim); % Variance of the process noise w[k]
% R = 0.25*eye(n_msr,n_msr); % Variance of the measurement noise v[k]
% R_hat = 10*R;
% P0 = 10*eye(n_dim,n_dim);
% use_analytic_jacob = false;


%% ======= Falling object ========
dt = 0.1;
tf = 30;
x0 = [300000; -20000; 0.001];
x_n = [1e5; 1e4; 1e-3];
x0 = x0 ./ x_n;
x0_hat = x0;
n_dim = length(x0);
n_msr = 1;
model = FallingObject(dt, x_n);
%% ======= Kalman filter params ========
Q = 0.0001*eye(n_dim,n_dim); % Variance of the process noise w[k]
R = 10000*eye(n_msr,n_msr); % Variance of the measurement noise v[k]
R_hat = 10*R;
P0 = diag([10 4 1]);
use_analytic_jacob = false;


