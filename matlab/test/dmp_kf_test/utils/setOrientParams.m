dt = 0.005;

est_goal = true;
est_tau = true;

Q0_offset = [1 0 0 0]';

% % speed: slower
% % target: x-y-z far
% Qg_offset = rotm2quat(rotx(95)*roty(110)*rotz(105))';
% time_offset = 2.5;

% % speed: much slower
% % target: x-y far, z near
% Qg_offset = rotm2quat(rotx(110)*roty(85)*rotz(20))';
% time_offset = 5;

% % speed: faster
% % target: x-y-z near
% Qg_offset = rotm2quat(rotx(15)*roty(35)*rotz(20))';
% time_offset = -2.5;

% % speed: much faster
% % target: x-z far, y near
% Qg_offset = rotm2quat(rotx(100)*roty(25)*rotz(95))';
% time_offset = -4;

% Qg_offset = rotm2quat(rotx(110)*roty(90)*rotz(10))';
% Qg_offset = rotm2quat(rotx(120)*roty(25)*rotz(95))';
% Qg_offset = rotm2quat(rotx(120)*roty(95)*rotz(25))';
% Qg_offset = rotm2quat(rotx(25)*roty(120)*rotz(95))';
% Qg_offset = rotm2quat(rotx(25)*roty(95)*rotz(120))';
% Qg_offset = rotm2quat(rotx(95)*roty(25)*rotz(120))';
% Qg_offset = rotm2quat(rotx(95)*roty(40)*rotz(50))';
% time_offset = 4;

tau_low_lim = 1.0;
tau_up_lim = 30.0; %Inf;

process_noise = diag([0.02 0.02 0.02 0.05]); % Q
msr_noise = 0.05^2;
msr_noise_hat = 10; %1000.0/dt; % R

% process_noise = process_noise*dt;
% msr_noise = msr_noise/dt;

init_params_variance = diag([1.0 1.0 1.0 10.0]); % P
a_pc = 0.99; % forgetting factor in fading memory KF
a_p = 1.002;

A_c = [];
b_c = [];
if (est_goal && est_tau)
    A_c = [0 0 0 -1; 0 0 0 1];
    b_c = [-1; 20];
elseif (est_tau)
    A_c = [-1; 1];
    b_c = [-1; 20];
end
enable_constraints = true;

num_diff_step = [0.001; 0.001; 0.001; 0.01];

plot_1sigma = false;

M_r = 1*eye(3,3);

params_ind = [];
if (est_goal), params_ind = [1 2 3]; end
if (est_tau), params_ind = [params_ind 4]; end

init_params_variance = init_params_variance(params_ind,params_ind);
process_noise = process_noise(params_ind,params_ind);
num_diff_step = num_diff_step(params_ind);

