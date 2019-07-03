
dt = 0.005;
tf = 5;

% x0 = [2;0];
% x0_hat = [4.0; 0.8]; % xhat[k|k-1]
% n_dim = length(x0);
% n_msr = 1;
% model = VDP(dt);

Q0 = [0.7 -0.2 -0.6 0.1]';
Q0 = Q0 / norm(Q0);
Qg = quatProd( rotm2quat(rotx(50)*rotz(110))', Q0);
vRot0 = [0; 0; 0];
x0 = [Q0; vRot0];
x0_hat = x0;
n_dim = length(x0);
n_msr = 7;
model = Quat5thPolyTrajTracker(Q0, Qg, tf, dt);


model = MackeyGlassTS(dt);


Q = 0.01*eye(n_dim,n_dim); % Variance of the process noise w[k]
R = 0.2*eye(n_msr,n_msr); % Variance of the measurement noise v[k]
R_hat = 10*R;
P0 = 10*eye(n_dim,n_dim);