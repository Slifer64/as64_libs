function sim_DMP_Pos_UKF_disc()

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

path = strrep(mfilename('fullpath'), 'sim_DMP_Pos_UKF_disc','');

load([path 'data/dmp_data.mat'],'dmp_p', 'Yg0', 'Y0', 'tau0');

Dim = length(Y0);
can_clock_ptr = dmp_p.can_clock_ptr;
can_clock_ptr.setTau(tau0);

%% DMP simulation
% set initial values
t = 0.0;
iters = 0;
Yg = Yg0;
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
Rn = eye(N_out,N_out) .* msr_noise;
Qn = eye(N_params,N_params) .* process_noise;

%% Set up UKF object
ukf = UKF(N_params, N_out, @pStateTransFun, @pMsrFun);
ukf.setProcessNoiseCov(Qn);
ukf.setMeasureNoiseCov(Rn);
a_p = exp(a_pc*dt);
% ukf.setFadingMemoryCoeff(a_p);
ukf.theta = theta;
ukf.P = P_theta;

ukf.enableParamsContraints(enable_constraints);
ukf.setParamsConstraints(A_c, b_c);

dmp_p.setY0(Y0);

disp('DMP-UKF (discrete) Pos simulation...');
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
    ddY = dmp_p.calcYddot(x, Y, dY, Yg);
    ddY_hat = dmp_p.calcYddot(x_hat, Y, dY, Yg_hat);

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
    ukf.correct(Y_out, pMsrCookie(dmp_p, t, Y, dY, Yg, tau, est_goal, est_tau));
    
    %% ========  KF time update  ========
    ukf.predict();

    theta = ukf.theta;
    P_theta = ukf.P;

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

function Y_out = pMsrFun(theta, cookie)

    if (cookie.est_goal), Yg_hat = theta(1:end-cookie.est_tau);
    else, Yg_hat = cookie.Yg;
    end
    
    if (cookie.est_tau), tau_hat = theta(end);
    else, tau_hat = cookie.tau;
    end

    x_hat = cookie.t/tau_hat;
    
    Y_out = cookie.dmp_p.calcYddot(x_hat, cookie.Y, cookie.dY, Yg_hat);

end

function cookie = pMsrCookie(dmp_p, t, Y, dY, Yg, tau, est_goal, est_tau)
    
    cookie = struct('dmp_p',dmp_p, 't',t, 'Y',Y, 'dY',dY, 'Yg',Yg, 'tau',tau, 'est_goal',est_goal, 'est_tau',est_tau);
    
end





