% clc;
% close all;
% clear;

format compact;
set_matlab_utils_path();


%% ###################################################################

path = strrep(mfilename('fullpath'), 'sim_DMPeo_EKFc_disc','');

load([path 'data/dmp_data.mat'],'dmp_eo', 'Qgd', 'Q0d', 'tau0');

dmp_eo.setTau(tau0);


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


can_clock_ptr = dmp_eo.can_clock_ptr;
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

Time = [];
Q_data = [];
vRot_data = [];
dvRot_data = [];
x_data = [];
Qg_data = [];
tau_data = [];
F_data = [];
Sigma_theta_data = [];

Qg = quatProd(quatExp(qg_offset), Qgd);

t_end = tau0 + time_offset;
tau = t_end;
can_clock_ptr.setTau(tau);

tau_hat = tau0;
Qg_hat = Q0;
eo_hat = DMP_eo.quat2eo(Q, Qg_hat);
x_hat = t/tau_hat;

N_out = 3;
Y_out_hat = zeros(N_out,1);
Y_out = zeros(N_out,1);

theta = [eo_hat; tau_hat];

P_theta = diag(init_params_variance); % init_params_variance;
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

dmp_eo.setQ0(Q0);

disp('DMP-EKF (discrete) eo simulation...');
tic;
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

    % DMP simulation
    dmp_eo.setTau(tau_hat);
    dvRot_hat = dmp_eo.calcRotAccel(x_hat, Q, vRot, Qg_hat);
    dmp_eo.setTau(tau);

    dvRot = dmp_eo.calcRotAccel(x, Q, vRot, Qg) + Sigma_vn*randn(N_out,1);

    F_ext = M_r .* (dvRot - dvRot_hat);

    Y_out = dvRot;
    % Y_out_hat = dvRot_hat;

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
    msr_cookie = getOrientMsrCookie(dmp_eo, t, Q, vRot);
    ekf.correct(Y_out, msr_cookie);

    % ========  KF time update  ========
    state_trans_cookie = getOrientStateTransCookie(vRot, dt);
    ekf.predict(state_trans_cookie);

    theta = ekf.theta;
    P_theta = ekf.P;

    % Numerical integration
    t = t + dt;
    x = x + dx*dt;
    Q = quatProd( quatExp(vRot*dt), Q );
    vRot = vRot + dvRot*dt;

    eo_hat = theta(1:3);
    Qg_hat = quatProd( quatExp(eo_hat), Q);
    tau_hat = theta(4);
    x_hat = t/tau_hat;
end
fprintf('Elapsed time: %.2f sec\n', toc);


%% Transform quaternion data
qg_data = zeros(3,size(Qg_data,2));
q_data = zeros(3,size(Qg_data,2));
for j=1:size(qg_data,2)
   qg_data(:,j) = quatLog( quatDiff(Qg, Qg_data(:,j)) );
   q_data(:,j) = quatLog( quatDiff(Qg, Q_data(:,j)) );
end
qg = quatLog( quatDiff(Qg, Qg) );
plot_DMP_EKFc_results(Time, qg_data, tau_data, Sigma_theta_data, q_data, vRot_data, qg, F_data, 'orient');


%% ======================================================================
%% ======================================================================


function ck = getOrientStateTransCookie(vRot, Ts)

    ck = struct('vRot',vRot, 'Ts',Ts);
    
end

function ck = getOrientMsrCookie(dmp, t, Q, vRot)

    ck = struct('dmp',dmp, 't',t, 'Q',Q, 'vRot',vRot);
    
end

function theta_next = oStateTransFun(theta, ck)

  theta_next = zeros(size(theta));
  
  eo = theta(1:3);
  Qe = quatExp(eo);
  rotVel = ck.vRot;
  deo = DMP_eo.rotVel2deo(rotVel, Qe);

  theta_next(1:3) = eo + deo*ck.Ts;
  theta_next(4) = theta(4);

end

function Y_out = oMsrFun(theta, ck)

  Q = ck.Q;
  eo_hat = theta(1:3);
  Qg_hat = quatProd( quatExp(eo_hat), Q);
  tau_hat = theta(4);
  x_hat = ck.t / tau_hat;

  tau0 = ck.dmp.getTau();
  ck.dmp.setTau(tau_hat);
  Y_out = ck.dmp.calcRotAccel(x_hat, Q, ck.vRot, Qg_hat);
  ck.dmp.setTau(tau0); % restore previous tau

end