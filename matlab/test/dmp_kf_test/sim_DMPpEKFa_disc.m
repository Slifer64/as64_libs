function sim_DMPpEKFa_disc()

%% ==============================================================

% clc;
% close all;
% clear;

format compact;
set_matlab_utils_path();

%% ###################################################################

rng(0);

dt = 0.005;

Y0_offset = [0 0 0]';
pg_offset = [0.8, -0.9, 0.7]';
time_offset = 5; 

% Qn = diag( [ 0.02*[1 1 1], 0.02*[1 1 1], 0.05*[1 1 1], 0.05*[1] ] );
% Rn = diag( [ 0.05^2*[1 1 1], 0.1^2*[1 1 1] ] );
% Rn_hat = diag( [ 500^2*[1 1 1], 1000*[1 1 1] ] );

Qn = 0.001*eye(10,10);
Rn = 0.1^2*eye(6,6);
Rn_hat = 100*eye(6,6);

P0 = diag( [ 1*[1 1 1], 1*[1 1 1], 100*[1 1 1], 1000*[1] ] );
a_p = 1.002;

plot_1sigma = false;

Mr = 5*eye(3,3);
Dr = 80*eye(3,3);
Kr = 400*eye(3,3);

%% ###################################################################

path = strrep(mfilename('fullpath'), 'sim_DMPpEKFa_disc','');

load([path 'data/dmp_data.mat'],'dmp_p', 'Yg0', 'Y0', 'tau0');

Dim = 3;
can_clock_ptr = dmp_p.can_clock_ptr;
can_clock_ptr.setTau(tau0);

%% DMP simulation
% set initial values
t = 0.0;
iters = 0;
x = 0.0; dx = 0.0;
p0 = Y0 + Y0_offset;
p = p0; p_dot = zeros(Dim,1); p_ddot = zeros(Dim,1);
F_ext = zeros(Dim,1);

Time = [];
p_data = []; p_dot_data = []; p_ddot_data = [];
p_hat_data = []; p_dot_hat_data = []; p_ddot_hat_data = [];
x_data = [];
pg_data = [];
tau_data = [];
F_data = [];
Sigma_theta_data = [];

pg = Yg0 + pg_offset;

t_end = tau0 + time_offset;
tau = t_end;
can_clock_ptr.setTau(tau);

tau_hat = tau0;
pg_hat = Yg0;
x_hat = t/tau_hat;
p_hat = p;
p_dot_hat = p_dot;
p_ddot_hat = zeros(3,1);

theta = [p_dot_hat; p_hat; pg_hat; tau_hat];
P_theta = P0;

Sigma_vn = sqrt(Rn);
N_out = 6;

%% Set up EKF object
ekf = DMPpEKFa(dmp_p, dt);
ekf.setProcessNoiseCov(Qn);
ekf.setMeasureNoiseCov(Rn_hat);
ekf.setFadingMemoryCoeff(a_p);
ekf.theta = theta;
ekf.P = P_theta;
% ekf.enableParamsContraints(enable_constraints);
% ekf.setParamsConstraints(A_c, b_c);
% ekf.setPartDerivStep(num_diff_step);
% ekf.setStateTransFunJacob(@pStateTransFunJacob);
% ekf.setMsrFunJacob(@pMsrFunJacob);

dmp_p.setY0(p0);

disp('DMPpEKFa simulation...');
tic
while (true)

    %% data logging

    Time = [Time t];

    p_data = [p_data p];
    p_dot_data = [p_dot_data p_dot];  
    p_ddot_data = [p_ddot_data p_ddot];
    
    p_hat_data = [p_hat_data p_hat];
    p_dot_hat_data = [p_dot_hat_data p_dot_hat];  
    p_ddot_hat_data = [p_ddot_hat_data p_ddot_hat];

    pg_data = [pg_data pg_hat];
    tau_data = [tau_data tau_hat];

    x_data = [x_data x];

    F_data = [F_data F_ext];

    Sigma_theta_data = [Sigma_theta_data sqrt(diag(P_theta))];

    %% DMP simulation  
    dmp_p.setTau(tau_hat);
    p_ddot_hat = dmp_p.calcYddot(x_hat, p_hat, p_dot_hat, pg_hat);
    
    dmp_p.setTau(tau);
    p_ddot = dmp_p.calcYddot(x, p, p_dot, pg);

    F_ext = Mr*(p_ddot - p_ddot_hat) + Dr*(p_dot - p_dot_hat) + Kr*(p - p_hat);

    
    Y_out = [p_dot; p] + Sigma_vn*randn(N_out,1);
    % Y_out_hat = [p_dot_hat; p_hat];

    %% Update phase variable
    dx = can_clock_ptr.getPhaseDot(x);

    %% Stopping criteria
    err_p = norm(pg-p)/norm(pg);
    if (err_p <= 0.5e-2 && t>=t_end)
        break; 
    end

    iters = iters + 1;
    if (t>=t_end)
        warning('Time limit reached. Stopping simulation...\n');
        break;
    end

    
    %% ========  KF measurement update  ========
    ekf.correct(Y_out);

    t = t + dt;

    %% ========  KF time update  ========
    ekf.predict(pStateTransCookie(t,p0));
    
    theta = ekf.theta;
    P_theta = ekf.P;

    %% Numerical integration
    x = x + dx*dt;
    p = p + p_dot*dt;
    p_dot = p_dot + p_ddot*dt;
    
    p_dot_hat = theta(1:3);
    p_hat = theta(4:6); 
    pg_hat = theta(7:9);
    tau_hat = theta(10);
    
    x_hat = t/tau_hat;

end
toc

figure
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, p_data(i,:), 'LineWidth',2, 'Color','blue');
   plot(Time, p_hat_data(i,:), 'LineWidth',2, 'Color','magenta');
   if (i==1), legend('actual','estimate'); end
   if (i==1), title('Position'); end
   hold off;
end

figure
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, p_dot_data(i,:), 'LineWidth',2, 'Color','blue');
   plot(Time, p_dot_hat_data(i,:), 'LineWidth',2, 'Color','magenta');
   if (i==1), legend('actual','estimate'); end
   if (i==1), title('Velocity'); end
   hold off;
end

figure
hold on;
for i=1:10
    plot(Time, Sigma_theta_data(i,:), 'LineWidth',2.0);
end
legend({'$\dot{p}_x$','$\dot{p}_y$','$\dot{p}_z$','$p_x$','$p_y$','$p_z$','$g_x$','$g_y$','$g_z$','$\tau$'}, 'interpreter','latex', 'fontsize',15);
title('$\sigma_{\theta}$', 'interpreter','latex', 'fontsize',15);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
hold off;

plot_pos_estimation_results(Time, pg, pg_data, tau, tau_data, Sigma_theta_data, F_data, plot_1sigma, p_data, p_dot_data);

end


function cookie = pStateTransCookie(t,p0)
    
    cookie = struct('t',t, 'p0',p0);
    
end




