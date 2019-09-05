function sim_DMPo_EKFa_matlab()

%% ==============================================================

% clc;
% close all;
% clear;

format compact;
set_matlab_utils_path();


%% ###################################################################

rng(0);

dt = 0.005;

Q0_offset = quatExp([0 0 0]');
Qg_offset = quatExp([0.8 -0.7 0.55]');
time_offset = 5;

% Qn = diag( [ 0.02*[1 1 1], 0.02*[1 1 1], 0.05*[1 1 1], 0.05*[1] ] );
% Rn = diag( [ 0.05^2*[1 1 1], 0.1^2*[1 1 1] ] );
% Rn_hat = diag( [ 500^2*[1 1 1], 1000*[1 1 1] ] );

Qn = 0.0001*eye(10,10);
Rn = 0.01^2*eye(6,6);
Rn_hat = diag([0.1*[1 1 1] 0.01*[1 1 1]]);

P0 = diag( [ 0.1*[1 1 1], 0.1*[1 1 1], 1*[1 1 1], 10*[1] ] );
a_p = 1.002;

plot_1sigma = false;

Mr = 1*eye(3,3);
Dr = 5*eye(3,3);
Kr = 10*eye(3,3);

%% ###################################################################


path = strrep(mfilename('fullpath'), 'sim_DMPo_EKFa_matlab','');

load([path 'data/dmp_data.mat'],'dmp_o', 'Qg0', 'Q0', 'tau0');

can_clock_ptr = dmp_o.can_clock_ptr;
dmp_o.setTau(tau0);

%% DMP simulation
% set initial values
t = 0.0;
iters = 0;
x = 0.0; dx = 0.0;
Q0 = quatProd(Q0_offset, Q0);
Q = Q0; vRot = zeros(3,1); dvRot = zeros(3,1);
F_ext = zeros(3,1);

Time = [];
Q_data = []; vRot_data = []; dvRot_data = [];
Q_hat_data = []; vRot_hat_data = []; dvRot_hat_data = [];
x_data = [];
Qg_data = [];
tau_data = [];
F_data = [];
Sigma_theta_data = [];

Qg = quatProd(Qg_offset, Qg0);

t_end = tau0 + time_offset;
tau = t_end;
can_clock_ptr.setTau(tau);

tau_hat = tau0;
Qg_hat = Qg0;
eQg_hat = quatLog(Qg_hat);
x_hat = t/tau_hat;
Q_hat = Q;
eQ_hat = quatLog(Q_hat);
vRot_hat = vRot;
dvRot_hat = zeros(3,1);

theta = [vRot_hat; eQ_hat; eQg_hat; tau_hat];
P_theta = P0;

Sigma_vn = sqrt(Rn);
N_out = 6;

%% Set up EKF object
ekf = extendedKalmanFilter(@oStateTransFun, @oMsrFun, theta);
ekf.State = theta;
ekf.StateCovariance = P_theta;
ekf.ProcessNoise = Qn;
ekf.MeasurementNoise = Rn_hat;

dmp_o.setQ0(Q0);

disp('DMPoEKFa matlab simulation...');
tic
while (true)

    %% data logging

    Time = [Time t];

    Q_data = [Q_data Q];
    vRot_data = [vRot_data vRot];  
    dvRot_data = [dvRot_data dvRot];
    
    Q_hat_data = [Q_hat_data Q_hat];
    vRot_hat_data = [vRot_hat_data vRot_hat];  
    dvRot_hat_data = [dvRot_hat_data dvRot_hat];

    Qg_data = [Qg_data Qg_hat];
    tau_data = [tau_data tau_hat];

    x_data = [x_data x];

    F_data = [F_data F_ext];

    Sigma_theta_data = [Sigma_theta_data sqrt(diag(P_theta))];

    %% DMP simulation
    dmp_o.setTau(tau_hat);
    dvRot_hat = dmp_o.calcRotAccel(x_hat, Q_hat, vRot_hat, Qg_hat);
    
    dmp_o.setTau(tau);
    dvRot = dmp_o.calcRotAccel(x, Q, vRot, Qg);

    F_ext = Mr*(dvRot - dvRot_hat) + Dr*(vRot - vRot_hat) + Kr*quatLog(quatDiff(Q,Q_hat));

    Y_out = [vRot; quatLog(Q)] + Sigma_vn*randn(N_out,1);

    %% Update phase variable
    dx = dmp_o.phaseDot(x);

    %% Stopping criteria
    err_p = quatLog(quatDiff(Qg,Q));
    if (err_p <= 0.5e-2 & t>=t_end)
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
    
    %% ========   KF time update  ===========
    ekf.predict(oStateTransCookie(dmp_o,t,dt));
    ekf.StateCovariance = a_p^2*(ekf.StateCovariance - Qn) + Qn;

    theta = ekf.State;
    P_theta = ekf.StateCovariance;

    %% ========   Numerical integration  ===========
    x = x + dx*dt;
    Q = quatProd( quatExp(vRot*dt), Q);
    vRot = vRot + dvRot*dt;
    
    vRot_hat = theta(1:3);
    eQ_hat = theta(4:6);
    Q_hat = quatExp(eQ_hat);
    eQg_hat = theta(7:9);
    Qg_hat = quatExp(eQg_hat);
    tau_hat = theta(10);
    
    x_hat = t/tau_hat;

end
toc

figure
for i=1:4
   subplot(4,1,i);
   hold on;
   plot(Time, Q_data(i,:), 'LineWidth',2, 'Color','blue');
   plot(Time, Q_hat_data(i,:), 'LineWidth',2, 'Color','magenta');
   if (i==1), legend('actual','estimate'); end
   if (i==1), title('Orientation (Quaternion)'); end
   hold off;
end

figure
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, vRot_data(i,:), 'LineWidth',2, 'Color','blue');
   plot(Time, vRot_hat_data(i,:), 'LineWidth',2, 'Color','magenta');
   if (i==1), legend('actual','estimate'); end
   if (i==1), title('Rotational Velocity'); end
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

plot_orient_estimation_results(Time, Qg, Qg_data, tau, tau_data, Sigma_theta_data, F_data, plot_1sigma, Q_data, vRot_data);

end

function cookie = oStateTransCookie(dmp, t, Ts)
    
    cookie = struct('t',t, 'Ts',Ts, 'dmp',dmp);
    
end

function theta_next = oStateTransFun(theta, cookie)

    t = cookie.t;
    dmp = cookie.dmp;
    Ts = cookie.Ts;
    
    vRot = theta(1:3);
    eQ = theta(4:6);
    eg = theta(7:9);
    tau = theta(10);

    Q = quatExp(eQ);
    Qg = quatExp(eg);

    x = t/tau;
    tau0 = dmp.getTau();
    dmp.setTau(tau);
    dvRot = dmp.calcRotAccel(x, Q, vRot, Qg);
    deQ = rotVel2deo(vRot, Q);
    dmp.setTau(tau0);

    theta_next = zeros(10,1);
    theta_next(1:3) = vRot + dvRot*Ts;
    theta_next(4:6) = eQ + deQ*Ts;
    theta_next(7:10) = theta(7:10);
            
end

function deo = rotVel2deo(rotVel, Q)
            
    J_deo_dQ = DMP_eo.jacobDeoDquat(Q);
    deo = 0.5*J_deo_dQ * quatProd([0; rotVel], Q);

end

function z = oMsrFun(theta)

    z = theta(1:6);

end


