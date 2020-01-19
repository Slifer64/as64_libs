dt = 0.005;

est_goal = true;
est_tau = true;

Y0_offset = [0.0 0.0 0.0]';

% % speed: slower
% % target: x-y-z far
% Yg_offset = [0.5, -0.4, 0.6]';
% time_offset = 2.5;

% % speed: much slower
% % target: x-y far, z near
% Yg_offset = [-0.65, 0.7, -0.2]';
% time_offset = 5;

% % speed: faster
% % target: x-y-z near
% Yg_offset = [0.2, -0.3, 0.15]';
% time_offset = -2.5;

% % speed: much faster
% % target: x-z far, y near
% Yg_offset = [0.45, 0.2, -0.45]';
% time_offset = -4;

Yg_offset = [0.75, -0.09, 0.64]';
time_offset = 6; 

Yg_up_lim = 0.8*[1.0 1.0 1.0]';
Yg_low_lim = -Yg_up_lim;
tau_low_lim = 1.0;
tau_up_lim = 30.0; %Inf;

process_noise = diag([0.02 0.02 0.02 0.05]); % Q
msr_noise = 0.1^2;
msr_noise_hat = 1000; %1000.0/dt; % R

% process_noise = process_noise*dt;
% msr_noise = msr_noise/dt;

% init_params_variance = diag([1.0 1.0 1.0 10.0]); % P
% assuming E(x-x_hat(0))=0.5 => cov(x-x_hat(0)) = 0.25
% assuming E(tau-tau_hat(0))=4 => cov(tau-tau_hat(0)) = 16
% init_params_variance = diag([0.25 0.25 0.25 16]); % P
init_params_variance = diag(1e5*[1 1 1 1]); % P
a_pc = 0.99; % forgetting factor in fading memory KF
a_p = 1.00002;

theta_low_lim = [];
theta_up_lim = [];
if (est_goal)
    theta_low_lim = Yg_low_lim;
    theta_up_lim = Yg_up_lim;
end
if (est_tau)
    theta_low_lim = [theta_low_lim; tau_low_lim];
    theta_up_lim = [theta_up_lim; tau_up_lim];
end
N_params = length(theta_low_lim);
A_c = [-eye(N_params, N_params); eye(N_params, N_params)];
b_c = [-theta_low_lim; theta_up_lim];
enable_constraints = true;

num_diff_step = [0.001; 0.001; 0.001; 0.01];

plot_1sigma = false;

M_r = 5*eye(3,3);

params_ind = [];
if (est_goal), params_ind = [1 2 3]; end
if (est_tau), params_ind = [params_ind 4]; end

init_params_variance = init_params_variance(params_ind,params_ind);
process_noise = process_noise(params_ind,params_ind);
num_diff_step = num_diff_step(params_ind);

