function sim_DMPp_EKFc_disc()

%% ==============================================================
%% DMP with state reset (y=y_r, y_dot=y_r_dot) and force feedback, i.e. 
%% y_ddot = h(theta, y_r, y_r_dot, t) + f_ext/M

% clc;
% close all;
% clear;

format compact;
set_matlab_utils_path();

%% ###################################################################

setPosParams();

path = strrep(mfilename('fullpath'), 'sim_DMPp_EKFc_disc','');

load([path 'data/dmp_data.mat'],'dmp_p', 'Yg0', 'Y0', 'tau0');

Dim = length(Y0);
can_clock_ptr = dmp_p.can_clock_ptr;
can_clock_ptr.setTau(tau0);

%% DMP simulation
% set initial values
t = 0.0;
iters = 0;
x = 0.0; dx = 0.0;
Y0 = Y0 + Y0_offset;
Y = Y0; dY = zeros(Dim,1); ddY = zeros(Dim,1);
F_ext = zeros(Dim,1);

Time = [];
Y_data = []; dY_data = []; ddY_data = [];
x_data = [];
Yg_data = [];
tau_data = [];
F_data = [];
Sigma_theta_data = [];

Yg = Yg0 + Yg_offset;

t_end = tau0 + time_offset;
tau = t_end;
can_clock_ptr.setTau(tau);

tau_hat = tau0;
Yg_hat = Yg0;
if (~est_goal), Yg_hat = Yg; end
if (~est_tau), tau_hat = tau; end
x_hat = t/tau_hat;

N_out = length(Yg_hat);
Y_out_hat = zeros(N_out,1);
Y_out = zeros(N_out,1);

theta = [];
if (est_goal),theta = Yg_hat; end
if (est_tau), theta = [theta; tau_hat]; end

N_params = length(theta);
P_theta = eye(N_params, N_params) .* init_params_variance;
Rn_hat = eye(N_out,N_out) .* msr_noise_hat;
Qn = eye(N_params,N_params) .* process_noise;

rng(0);
Rn = eye(N_out,N_out) .* msr_noise;
Sigma_vn = sqrt(Rn);


%% Set up EKF object
ekf = EKF(N_params, N_out, @pStateTransFun, @pMsrFun);
ekf.setProcessNoiseCov(Qn);
ekf.setMeasureNoiseCov(Rn_hat);
% a_p = exp(a_pc*dt);
ekf.setFadingMemoryCoeff(a_p);
ekf.theta = theta;
ekf.P = P_theta;

ekf.enableParamsContraints(enable_constraints);
ekf.setParamsConstraints(A_c, b_c);
ekf.setPartDerivStep(num_diff_step);

ekf.setStateTransFunJacob(@pStateTransFunJacob);
ekf.setMsrFunJacob(@pMsrFunJacob);

dmp_p.setY0(Y0);

disp('DMP-EKF (discrete) Pos simulation...');
tic
while (true)

    %% data logging

    Time = [Time t];

    Y_data = [Y_data Y];
    dY_data = [dY_data dY];  
    ddY_data = [ddY_data ddY];

    Yg_data = [Yg_data Yg_hat];
    tau_data = [tau_data tau_hat];

    x_data = [x_data x];

    F_data = [F_data F_ext];

    Sigma_theta_data = [Sigma_theta_data sqrt(diag(P_theta))];

    %% DMP simulation  
    dmp_p.setTau(tau_hat);
    ddY_hat = dmp_p.calcYddot(x_hat, Y, dY, Yg_hat);
    
    dmp_p.setTau(tau);
    ddY = dmp_p.calcYddot(x, Y, dY, Yg) + Sigma_vn*randn(N_out,1);

    F_ext = M_r * (ddY - ddY_hat);
    
    Y_out = ddY;
    % Y_out_hat = ddY_hat;

    %% Update phase variable
    dx = can_clock_ptr.getPhaseDot(x);

    %% Stopping criteria
    err_p = norm(Yg-Y)/norm(Yg);
    if (err_p <= 0.5e-2 && t>=t_end)
        break; 
    end

    iters = iters + 1;
    if (t>=t_end)
        warning('Time limit reached. Stopping simulation...\n');
        break;
    end

    %% ========  KF measurement update  ========
    ekf.correct(Y_out, pMsrCookie(dmp_p, t, Y, dY, Y0, Yg, tau, est_goal, est_tau));
    
    %% ========  KF time update  ========
    ekf.predict();

    theta = ekf.theta;
    P_theta = ekf.P;

    %% Numerical integration
    t = t + dt;
    x = x + dx*dt;
    Y = Y + dY*dt;
    dY = dY + ddY*dt;
    
    if (est_goal), Yg_hat = theta(1:end-est_tau); end
    if (est_tau), tau_hat = theta(end); end;
    x_hat = t/tau_hat;

end
toc

plot_pos_estimation_results(Time, Yg, Yg_data, tau, tau_data, Sigma_theta_data, F_data, plot_1sigma, Y_data, dY_data);

end

function theta_next = pStateTransFun(theta, cookie)
    
    theta_next = theta;
    
end

function J = pStateTransFunJacob(theta, cookie)
    
    n_params = length(theta);
    J = eye(n_params, n_params);
    
end

function Y_out = pMsrFun(theta, cookie)

    if (cookie.est_goal), Yg_hat = theta(1:end-cookie.est_tau);
    else, Yg_hat = cookie.Yg;
    end
    
    if (cookie.est_tau), tau_hat = theta(end);
    else, tau_hat = cookie.tau;
    end

    x_hat = cookie.t/tau_hat;
    
    cookie.dmp_p.setTau(tau_hat);
    Y_out = cookie.dmp_p.calcYddot(x_hat, cookie.Y, cookie.dY, Yg_hat);
    cookie.dmp_p.setTau(cookie.tau);

end

function J = pMsrFunJacob(theta, cookie)
    
    if (cookie.est_goal), Yg_hat = theta(1:end-cookie.est_tau);
    else, Yg_hat = cookie.Yg;
    end
    
    if (cookie.est_tau), tau_hat = theta(end);
    else, tau_hat = cookie.tau;
    end
    
    x_hat = cookie.t / tau_hat;

    J = cookie.dmp_p.getAcellPartDev_g_tau(cookie.t, cookie.Y, cookie.dY, cookie.Y0, x_hat, Yg_hat, tau_hat);

end

function cookie = pMsrCookie(dmp_p, t, Y, dY, Y0, Yg, tau, est_goal, est_tau)
    
    cookie = struct('dmp_p',dmp_p, 't',t, 'Y',Y, 'dY',dY, 'Y0',Y0, 'Yg',Yg, 'tau',tau, 'est_goal',est_goal, 'est_tau',est_tau);
    
end





