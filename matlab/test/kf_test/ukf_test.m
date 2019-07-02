clc;
% close all;
clear;

set_matlab_utils_path();

dt = 0.005;
vdp = VDP(dt);

% Your initial state guess at time k, utilizing measurements up to time k-1: xhat[k|k-1]
x0_hat = [4.0; 0.8]; % xhat[k|k-1]
Q = 1*diag([0.01 0.01]); % Variance of the process noise w[k]
R = 0.002; % Variance of the measurement noise v[k]
R_hat = 10*R;
P0 = 1*diag([1 1]);

% Construct the filter
n_params = length(x0_hat);
n_msr = length(vdp.msrFun(x0_hat));
ukf = UKF(n_params, n_msr, @vdp.stateTransFun, @vdp.msrFun);
ukf.theta = x0_hat;
ukf.P = P0;
ukf.setProcessNoiseCov(Q);
ukf.setMeasureNoiseCov(R_hat);
ukf.setFadingMemoryCoeff(1.0);
ukf.alpha = 1e-3;
ukf.beta = 2;
ukf.k = 0;

Time = 0:dt:5;
x_data = [];
x0 = [2;0];
x = x0;
t = 0;
for i=1:length(Time)  
    x_data = [x_data x];
    dx = vdp.stateTransFunCont(x);
    
    t = t + dt;
    x = x + dx*dt;
end

rng(1); % Fix the random number generator for reproducible results
n_out = length(vdp.msrFun(x_data(:,1)));
y_nn_data = zeros(n_out, length(Time));
for j=1:length(y_nn_data), y_nn_data(:,j) = vdp.msrFun(x_data(:,j)); end
y_data = y_nn_data + sqrt(R)*randn(size(y_nn_data)); % sqrt(R): Standard deviation of noise


%% ========================================================================
%% ========================================================================


n_steps = numel(y_data); % Number of time steps
e = zeros(n_steps,1); % Residuals (or innovations)

P_data = [];
x_hat_data = [];
P = ukf.P;
x_hat = ukf.theta;

% sigmap_colors = [[0, 1, 0]; [0.5, 1, 0.5]; [0.68, 1, 0.68]; [0.8, 1, 0.8]; [0.9, 1, 0.9]];
% thetam_colors = [[0, 0, 1]; [0.5, 0.5, 1]; [0.68, 0.68, 1]; [0.8, 0.8, 1]; [0.9, 0.9, 1]];
% thetap_colors = [[1, 0, 0]; [1, 0.5, 0.5]; [1, 0.68, 0.68]; [1, 0.8, 0.8]; [1, 0.9, 0.9]];
% N_p = length(sigmap_colors);

N_sigma = 2*length(x_hat) + 1;

figure;
ax_tim = subplot(1,2,1);
ax_msr = subplot(1,2,2);
time_up_plot = UKFTimeUpdatePlot(ax_tim, N_sigma);
msr_up_plot = UKFMsrUpdatePlot(ax_msr, N_sigma);

for k=1:n_steps
    
    x_hat_data = [x_hat_data x_hat];
    P_data = [P_data P(:)];
    
    % Residuals (or innovations): Measured output - Predicted output
    e(k) = y_data(k) - vdp.msrFun(ukf.theta); % ukf.State is x[k|k-1] at this point
    
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [theta_p, theta_m, Sigma_points] = ukf.correct(y_data(k));
    
    msr_up_plot.update(theta_p, theta_m, Sigma_points);
    msr_up_plot.rescale();

    P = ukf.P;
    x_hat = ukf.theta;

    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    [theta_next, Sigma_points_next] = ukf.predict();
    
    time_up_plot.update(x_hat, theta_next, Sigma_points_next);
    time_up_plot.rescale();
   
%     delete(ax_time.Children);
%     for j=1:size(Sigma_points,2)
%         if (j==2), hold(ax_time,'on'); end
%         pl = scatter(Sigma_points_next(1,j), Sigma_points_next(2,j), 'SizeData',100, 'LineWidth',3, 'Marker','x', 'MarkerEdgeColor','blue', 'Parent',ax_time, 'HandleVisibility','off');
%         title(ax_time, 'Time update', 'interpreter','latex', 'fontsize',17, 'Color',[0 0 1], 'BackgroundColor',[1 1 1], 'EdgeColor',[0 1 0]); 
%     end
%     pl.HandleVisibility = 'on';
%     scatter(theta_next(1), theta_next(2), 'SizeData',100, 'LineWidth',3, 'Marker','o', 'MarkerEdgeColor','cyan', 'Parent',ax_time);
%     legend(ax_time,{'$x_{\sigma}$', '$x_{k+1}^{-}$'}, 'interpreter','latex', 'fontsize',15, 'location','northeastoutside');
%     hold(ax_time,'off');
    
    t = Time(k)
    
    pause(0.01)
end

%% ========================================================================
%% ========================================================================

figure
plot(Time, e, 'LineWidth',2.0, 'Color','magenta');
title('Innovation error', 'interpreter','latex', 'fontsize',17);

figure('Position',[282 128 1436 846]);
subplot(2,2,1);
hold on;
plot(Time,x_data(1,:), 'LineWidth',2.0, 'Color','blue');
plot(Time,x_hat_data(1,:), 'LineWidth',2.0, 'Color','magenta');
legend({'$x_1$','$\hat{x}_1$'}, 'interpreter','latex', 'fontsize',15);
ylabel('$x_1$', 'interpreter','latex', 'fontsize',17);
title('UKF estimation results', 'interpreter','latex', 'fontsize',17);
hold off;
subplot(2,2,3);
hold on;
plot(Time,x_data(2,:), 'LineWidth',2.0, 'Color','blue');
plot(Time,x_hat_data(2,:), 'LineWidth',2.0, 'Color','magenta');
legend({'$x_2$','$\hat{x}_2$'}, 'interpreter','latex', 'fontsize',15);
xlabel('Time [$s$]', 'interpreter','latex', 'fontsize',15);
ylabel('$x_2$', 'interpreter','latex', 'fontsize',17);
hold off;

x_err_data = x_data-x_hat_data;
subplot(2,2,2);
hold on;
plot(Time,x_err_data(1,:), 'LineWidth',2.0, 'Color','blue');  % Error for the first state
plot(Time,sqrt(P_data(1,:)), 'LineWidth',2.0, 'Color','red');  % 1-sigma upper-bound
plot(Time,-sqrt(P_data(1,:)), 'LineWidth',2.0, 'Color','red');  % 1-sigma lower-bound
xlabel('Time [$s$]', 'interpreter','latex', 'fontsize',15);
ylabel('$\tilde{x}_1$', 'interpreter','latex', 'fontsize',17);
legend({'State estimate','$\pm \sigma$'}, 'interpreter','latex', 'fontsize',15);
title('State estimation errors', 'interpreter','latex', 'fontsize',17);
hold off;
subplot(2,2,4);
hold on;
plot(Time,x_err_data(2,:), 'LineWidth',2.0, 'Color','blue');  % Error for the first state
plot(Time,sqrt(P_data(3,:)), 'LineWidth',2.0, 'Color','red');  % 1-sigma upper-bound
plot(Time,-sqrt(P_data(3,:)), 'LineWidth',2.0, 'Color','red');  % 1-sigma lower-bound
xlabel('Time [$s$]', 'interpreter','latex', 'fontsize',15);
ylabel('$\tilde{x}_2$', 'interpreter','latex', 'fontsize',17);
hold off;



%% ========================================================================
%% ========================================================================
