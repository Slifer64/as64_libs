% clc;
% close all;
% clear;

format compact;
set_matlab_utils_path();


%% ###################################################################

path = strrep(mfilename('fullpath'), 'sim_DMPo_EKFc_disc','');

load([path 'data/dmp_data.mat'],'dmp_o', 'Qgd', 'Q0d', 'tau0');

dmp_o.setTau(tau0);

%% ###################################################################

dt = 0.005; %  numerical integration time step
q0_offset = [0; 0; 0]; %  initial orientation offset, as quatLog
qg_offset = [0.4; 0.3; -0.35]; %  target orientation offset, as quatLog
time_offset = -2.5; %  time scaling offset

Yg_low_lim = 2*[-pi; -pi; -pi]; % lower limit for position estimate
Yg_up_lim = 2*[pi; pi; pi]; % upper limit for position estimate
tau_low_lim = 1.0; %  upper limit for tau estimate
tau_up_lim = 30.0; %  lower limit for tau estimate

theta_low_lim = [Yg_low_lim; tau_low_lim];
theta_up_lim = [Yg_up_lim; tau_up_lim];

process_noise = [0.002; 0.002; 0.002; 0.005]; %  process noise covariance
msr_noise = 0.0025; %  measurement noise covariance for corrupting measurements in sim
msr_noise_hat = 10; %  measurement noise covariance used in EKF

init_params_variance = [1; 1; 1; 10]; %  P0
a_p = 1.001; %  forgetting factor in fading memory discrete EKF

num_diff_step = [0.001; 0.001; 0.001; 0.01]; %  num-diff step used in EKF for computing Jacobians
ekf_analytic_Jacob = false; %  use analytic Jacobians in EKF
enable_constraints = true;

M_r = [0.5; 0.5; 0.5]; %  reference model inertia

N_params = 4;
% A_c = [0,0,0,-1; 0,0,0,1];
% b_c = [-tau_low_lim; tau_up_lim];
A_c = [-eye(N_params, N_params); eye(N_params, N_params)];
b_c = [-theta_low_lim; theta_up_lim];

%% ###################################################################

can_clock_ptr = dmp_o.can_clock_ptr;
can_clock_ptr.setTau(tau0);

% DMP simulation
% set initial values
Dim = 3;
t = 0.0;
x = 0.0;
dx = 0.0;
Q0 = quatProd(quatExp(q0_offset), Q0d);
Q = Q0;
vRot = zeros(Dim,1);
dvRot = zeros(Dim,1);
F_ext = zeros(Dim,1);
Q1 = DMPo.quatTf(Q, Q0);
q = quatLog(Q1);
q_dot = zeros(3,1);
Fq = zeros(3,1);

Time = [];
Q_data = [];
vRot_data = [];
dvRot_data = [];
x_data = [];
Qg_data = [];
tau_data = [];
F_data = [];
Sigma_theta_data = [];

Kg_data = [];
Kg = zeros(12,1);

Qg = quatProd(quatExp(qg_offset), Qgd);
qg = DMPo.quat2q(Qg, Q0);

t_end = tau0 + time_offset;
tau = t_end;
can_clock_ptr.setTau(tau);

tau_hat = tau0;
Qg_hat = Q0; % Qg0;
qg_hat = DMPo.quat2q(Qg_hat, Q0);
x_hat = t/tau_hat;

N_out = 3;
Y_out_hat = zeros(N_out,1);
Y_out = zeros(N_out,1);

theta = [qg_hat; tau_hat];

P_theta = eye(N_params, N_params); % init_params_variance;
Rn_hat = eye(N_out,N_out) * msr_noise_hat;
Qn = diag(process_noise); % process_noise;

rng(0);
Rn = eye(N_out,N_out) * msr_noise;
Sigma_vn = sqrt(Rn);

% Set up EKF object
ekf = EKF(N_params, N_out, @oStateTransFun, @oMsrFun);
ekf.theta = theta;
ekf.P = P_theta;
ekf.setProcessNoiseCov(Qn);
ekf.setMeasureNoiseCov(Rn_hat);
% a_p = exp(a_pc*dt);
ekf.setFadingMemoryCoeff(a_p);

ekf.enableParamsContraints(enable_constraints);
ekf.setParamsConstraints(A_c, b_c);
ekf.setPartDerivStep(num_diff_step);
ekf.setStateTransFunJacob(@oStateTransFunJacob);
if (ekf_analytic_Jacob), ekf.setMsrFunJacob(@oMsrFunJacob); end

dmp_o.setQ0(Q0);

disp('DMP-EKF (discrete) Orient simulation...\n')
tic
while (true)

    % data logging
    Time = [Time, t];
    Q_data = [Q_data, Q];
    vRot_data = [vRot_data, vRot];
    dvRot_data = [dvRot_data, dvRot];

    Qg_data = [Qg_data, Qg_hat];
    tau_data = [tau_data, tau_hat];

    x_data = [x_data, x];
    F_data = [F_data, F_ext];
    Sigma_theta_data = [Sigma_theta_data, sqrt(diag(P_theta))];

    Kg_data = [Kg_data, Kg];

    % DMP simulation
    dmp_o.setTau(tau_hat);
    q_ddot_hat = dmp_o.calcYddot(x_hat, q, q_dot, qg_hat);
    % dvRot_hat = dmp_o.calcRotAccel(x_hat, Q, vRot, Qg_hat);
    dmp_o.setTau(tau);

    F_noise = Sigma_vn*randn(N_out,1);
    Q1 = DMPo.quatTf(Q, Q0);
    F_noise = 0.5*DMPo.jacobqQ(Q1)*quatProd([0; F_noise], Q1);
    
    q_ddot = dmp_o.calcYddot(x, q, q_dot, qg) + F_noise;
    % q_ddot = dmp_o.calcYddot(x, q, q_dot, qg);
    % dvRot = dmp_o.calcRotAccel(x, Q, vRot, Qg);

    Fq = (q_ddot - q_ddot_hat);
    JQq = DMPo.jacobQq(Q1);
    F_ext = quatProd( 2*JQq*(q_ddot - q_ddot_hat), quatInv(Q1) );
    F_ext = M_r.*F_ext(2:4);

    % Fext2 = M_r*(dvRot-dvRot_hat);
    % Fext_err = norm(F_ext - Fext2);
    % if (Fext_err > 1e-6), Fext_err end

    Y_out = q_ddot;
    % Y_out_hat = q_ddot_hat;

    % Update phase variable
    dx = can_clock_ptr.getPhaseDot(x);

    % Stopping criteria
    err_o = norm( quatLog( quatProd(Qg,quatInv(Q)) ) );
    if (err_o <= 0.5e-2 && t>=t_end), break; end

    if (t>=t_end)
        fprintf('Time limit reached. Stopping simulation...\n');
        break;
    end

    % ========  KF measurement update  ========
    msr_cookie = getOrientMsrCookie(dmp_o, t, q, q_dot);
    ekf.correct(Y_out, msr_cookie);

    % ========  KF time update  ========
    ekf.predict();

    theta = ekf.theta;
    P_theta = ekf.P;
    
    Kg = ekf.K(:);

    % Numerical integration
    t = t + dt;
    x = x + dx*dt;
    q = q + q_dot*dt;
    q_dot = q_dot + q_ddot*dt;

    Q = DMPo.q2quat(q, Q0);
    Q1 = DMPo.quatTf(Q, Q0);
    vRot = DMPo.qdot2rotVel(q_dot, Q1);
    dvRot = DMPo.qddot2rotAccel(q_ddot, vRot, Q1);

    qg_hat = theta(1:3);
    Qg_hat = DMPo.q2quat(qg_hat, Q0);
    tau_hat = theta(4);
    x_hat = t/tau_hat;
end
fprintf('Elapsed time: %.2f sec\n', toc);


%% Transform quaternion data
qg_data = zeros(3,size(Qg_data,2));
q_data = zeros(3,size(Qg_data,2));
for j=1:size(qg_data,2)
   qg_data(:,j) = quatLog( quatDiff(Qg_data(:,j), Q0) );
   q_data(:,j) = quatLog( quatDiff(Q_data(:,j), Q0) );
end
qg = quatLog( quatDiff(Qg, Q0) );
plot_DMP_EKFc_results(Time, qg_data, tau_data, Sigma_theta_data, q_data, vRot_data, qg, F_data, 'orient');

% figure;
% plot(Kg_data');

%% ======================================================================
%% ======================================================================

function ck = getOrientMsrCookie(dmp, t, Y, dY)

    ck = struct('dmp',dmp, 't',t, 'Y',Y, 'dY',dY);

end


function theta_next = oStateTransFun(theta, ck)

    if (nargin < 2), ck = []; end
    theta_next = theta;
    
end
  
  
function Y_out = oMsrFun(theta, ck)

    Yg_hat = theta(1:3);
    tau_hat = theta(4);
    x_hat = ck.t/tau_hat;
    tau0 = ck.dmp.getTau();
    ck.dmp.setTau(tau_hat);
    Y_out = ck.dmp.calcYddot(x_hat, ck.Y, ck.dY, Yg_hat);
    ck.dmp.setTau(tau0); % restore previous value of tau

end


function J = oStateTransFunJacob(theta, ck)

    if (nargin < 2), ck = []; end
    n_params = length(theta);
    J = eye(n_params, n_params);

end


function J = oMsrFunJacob(theta, ck)

    Yg_hat = theta(1:end-1);
    tau_hat = theta(end);
    x_hat = ck.t / tau_hat;
    Y0 = zeros(size(Yg_hat));
    J = ck.dmp.getAcellPartDev_qg_tau(ck.t, ck.Y, ck.dY, Y0, x_hat, Yg_hat, tau_hat);
    
end

