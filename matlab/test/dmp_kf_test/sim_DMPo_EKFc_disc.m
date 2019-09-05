function sim_DMPo_EKFc_disc()

%% ==============================================================
%% DMP with state reset (y=y_r, y_dot=y_r_dot) and force feedback, i.e. 
%% y_ddot = h(theta, y_r, y_r_dot, t) + f_ext/M

% clc;
% close all;
% clear;

format compact;
set_matlab_utils_path();


%% ###################################################################

setOrientParams();

path = strrep(mfilename('fullpath'), 'sim_DMPo_EKFc_disc','');

load([path 'data/dmp_data.mat'],'dmp_o', 'Qg0', 'Q0', 'tau0');

dmp_o.setTau(tau0);

%% DMP simulation
% set initial values
t = 0.0;
iters = 0;
Qg = quatProd(Qg_offset, Qg0);
x = 0.0; dx = 0.0;
Q0 = quatProd(Q0_offset, Q0);
Q = Q0; vRot = zeros(3,1); dvRot = zeros(3,1);
F_ext = zeros(3,1);

Time = [];
Q_data = []; vRot_data = []; dvRot_data = [];
x_data = [];
Qg_data = [];
tau_data = [];
F_data = [];
Sigma_theta_data = [];

t_end = tau0 + time_offset;
tau = t_end;
dmp_o.setTau(tau);

tau_hat = tau0;
Qg_hat = Qg0;
if (~est_goal), Qg_hat = Qg; end
if (~est_tau), tau_hat = tau; end
eo_hat = DMP_eo.quat2eo(Q, Qg_hat);
x_hat = t/tau_hat;

N_out = 3;
theta = [];
if (est_goal),theta = eo_hat; end
if (est_tau), theta = [theta; tau_hat]; end

N_params = length(theta);
P_theta = eye(N_params, N_params) .* init_params_variance;
Rn_hat = eye(N_out,N_out) .* msr_noise_hat;
Qn = eye(N_params,N_params) .* process_noise;

rng(0);
Rn = eye(N_out,N_out) .* msr_noise;
Sigma_vn = sqrt(Rn);

%% Set up EKF object
ekf = EKF(N_params, N_out, @oStateTransFun, @oMsrFun);
ekf.setProcessNoiseCov(Qn);
ekf.setMeasureNoiseCov(Rn_hat);
% a_p = exp(a_pc*dt);
ekf.setFadingMemoryCoeff(a_p);
ekf.theta = theta;
ekf.P = P_theta;

ekf.enableParamsContraints(enable_constraints);
ekf.setParamsConstraints(A_c, b_c);
ekf.setPartDerivStep(num_diff_step);

ekf.setMsrFunJacob(@oMsrFunJacob);

dmp_o.setQ0(Q0);

disp('DMP-EKF (discrete) Orient simulation...')
tic
while (true)

    %% data logging

    Time = [Time t];

    Q_data = [Q_data Q];
    vRot_data = [vRot_data vRot];  
    dvRot_data = [dvRot_data dvRot];

    Qg_data = [Qg_data Qg_hat];
    tau_data = [tau_data tau_hat];

    x_data = [x_data x];

    F_data = [F_data F_ext];

    Sigma_theta_data = [Sigma_theta_data sqrt(diag(P_theta))];

    %% DMP simulation
    dmp_o.setTau(tau_hat);
    dvRot_hat = dmp_o.calcRotAccel(x_hat, Q, vRot, Qg_hat);
    
    dmp_o.setTau(tau);
    dvRot = dmp_o.calcRotAccel(x, Q, vRot, Qg) + Sigma_vn*randn(N_out,1);

    F_ext = M_r * (dvRot - dvRot_hat);
    Y_out = dvRot;

    %% Update phase variable
    dx = dmp_o.phaseDot(x);

    %% Stopping criteria
    err_p = norm(Qg-Q)/norm(Qg);
    if (err_p <= 0.5e-2 && t>=t_end)
        break; 
    end

    iters = iters + 1;
    if (t>=t_end)
        warning('Time limit reached. Stopping simulation...\n');
        break;
    end

    %% ========   KF measurement update (correction)  ===========
    ekf.correct(Y_out, oMsrCookie(dmp_o, t, Q, vRot, Qg, tau, est_goal, est_tau));
    
    %% ========   KF time update  ===========
    ekf.predict(oStateTransCookie(vRot, dt, est_goal, est_tau));

    theta = ekf.theta;
    P_theta = ekf.P;

    %% ========   Numerical integration  ===========
    t = t + dt;
    x = x + dx*dt;
    Q = quatProd( quatExp(vRot*dt), Q);
    vRot = vRot + dvRot*dt;
    
    if (est_goal)
        eo_hat = theta(1:end-est_tau); 
        Qg_hat = quatProd( quatExp(eo_hat), Q);
    end
    if (est_tau), tau_hat = theta(end); end;
    
    x_hat = t/tau_hat;

end
toc

plot_orient_estimation_results(Time, Qg, Qg_data, tau, tau_data, Sigma_theta_data, F_data, plot_1sigma, Q_data, vRot_data);

end

function theta_next = oStateTransFun(theta, cookie)
    
    theta_next = [];
    
    if (cookie.est_goal)
        eo = theta(1:end-cookie.est_tau);
        Qe = quatExp(eo);
        rotVel = cookie.vRot;
        deo = DMP_eo.rotVel2deo(rotVel, Qe);
        theta_next = eo + deo*cookie.Ts;
    end

    if (cookie.est_tau)
        theta_next = [theta_next; theta(end)]; 
    end
    
end

function Y_out = oMsrFun(theta, cookie)

    Q = cookie.Q;
    
    if (cookie.est_goal)
        eo_hat = theta(1:end-cookie.est_tau);
        Qg_hat = quatProd( quatExp(eo_hat), Q);
    else
        Qg_hat = cookie.Qg;
    end
    
    if (cookie.est_tau), tau_hat = theta(end); 
    else, tau_hat = cookie.tau;
    end;

    x_hat = cookie.t / tau_hat;
    
    cookie.dmp_o.setTau(tau_hat);
    Y_out = cookie.dmp_o.calcRotAccel(x_hat, Q, cookie.vRot, Qg_hat);
    cookie.dmp_o.setTau(cookie.tau);

end

function cookie = oStateTransCookie(vRot, Ts, est_goal, est_tau)
    
    cookie = struct('vRot',vRot, 'Ts',Ts, 'est_goal',est_goal, 'est_tau',est_tau);
    
end

function cookie = oMsrCookie(dmp_o, t, Q, vRot, Qg, tau, est_goal, est_tau)
    
    cookie = struct('dmp_o',dmp_o, 't',t, 'Q',Q, 'vRot',vRot, 'Qg',Qg, 'tau',tau, 'est_goal',est_goal, 'est_tau',est_tau);
    
end

function H_k = oMsrFunJacob(theta, cookie)
    
    % roll out cookie
    dmp_o = cookie.dmp_o;
    t = cookie.t;
    Q = cookie.Q;
    rotVel = cookie.vRot;
    
    % step for numerical calc of partial derivatives
    dtheta = [0.001; 0.001; 0.001; 0.01];
            
    % create the Jacobian
    N_params = length(theta);
    N_out = length(rotVel);
    H_k = zeros(N_out,N_params);

    % compute Jacobian w.r.t goal numerically
    dtheta_j = zeros(N_params,1);
    for j=1:N_params
        dtheta_j(j) = dtheta(j);
        Htheta2 = oMsrFun(theta + dtheta_j, cookie);
        Htheta1 = oMsrFun(theta - dtheta_j, cookie);
        H_k(:,j) = (Htheta2 - Htheta1) / (2*dtheta(j));
        dtheta_j(j) = 0.0;
    end
    
%     % compute Jacobian w.r.t. tau analytically
%     eo = theta(1:end-1);
%     tau_hat = theta(end);
%     x_hat = t/tau_hat;
%     Qe = quatExp(eo);
%     Qg = quatProd(Qe, Q);
%     eo0 = DMP_eo.quat2eo(Q0, Qg);
%     deo = DMP_eo.rotVel2deo(rotVel, Qe);
%     for i=1:3
%         C = dmp_o.dmp{i}.getAcellPartDev_g_tau(t, eo(i), deo(i), eo0(i), x_hat, 0, tau_hat);
%         H_k(i,end) = C(2);
%     end
    
end


