clc;
% close all;
clear;

set_matlab_utils_path();

setParams();

% Construct the filter
n_params = length(x0_hat);
ukf = UKF(n_params, n_msr, @model.stateTransFun, @model.msrFun);
ukf.theta = x0_hat;
ukf.P = P0;
ukf.setProcessNoiseCov(Q);
ukf.setMeasureNoiseCov(R_hat);
ukf.setFadingMemoryCoeff(1.0);
ukf.alpha = 1e-3;
ukf.beta = 2;
ukf.k = 0;

Time = 0:dt:tf;
x_data = [];
x = x0;
t = 0;
for i=1:length(Time)  
    x_data = [x_data x];
    
    model.setExtParams(t);
    x = model.stateTransFun(x);
    t = t + dt;
end

rng(1); % Fix the random number generator for reproducible results
n_out = length(model.msrFun(x_data(:,1)));
y_nn_data = zeros(n_out, length(Time));
for j=1:length(y_nn_data), y_nn_data(:,j) = model.msrFun(x_data(:,j)); end
y_data = y_nn_data + sqrt(R)*randn(size(y_nn_data)); % sqrt(R): Standard deviation of noise


%% ========================================================================
%% ========================================================================


n_steps = length(Time); % Number of time steps
e = zeros(n_msr, n_steps); % Residuals (or innovations)

P_data = [];
x_hat_data = [];
P = ukf.P;
x_hat = ukf.theta;

N_sigma = 2*length(x_hat) + 1;

% figure;
% ax_tim = subplot(1,2,1);
% ax_msr = subplot(1,2,2);
% time_up_plot = UKFTimeUpdatePlot(ax_tim, N_sigma);
% msr_up_plot = UKFMsrUpdatePlot(ax_msr, N_sigma);

% figure;
% x1_plot = PlotLines1D(subplot(1,2,1), 2);
% x1_plot.setLegend({'$x_1$','$\hat{x}_1$'}, 'interpreter','latex', 'fontsize',15);
% x1_plot.setYLabel('$x_1$', 'interpreter','latex', 'fontsize',18);
% x1_plot.setXLabel('time [$s$]', 'interpreter','latex', 'fontsize',16);
% x1_plot.setTitle('UKF estimation results', 'interpreter','latex', 'fontsize',17);
% x1_plot.setLineProperties(1, 'LineWidth',2.0, 'Color','blue');
% x1_plot.setLineProperties(2, 'LineWidth',2.0, 'Color','magenta');
% x2_plot = PlotLines1D(subplot(1,2,2), 2);
% x2_plot.setLegend({'$x_2$','$\hat{x}_2$'}, 'interpreter','latex', 'fontsize',15);
% x2_plot.setYLabel('$x_2$', 'interpreter','latex', 'fontsize',18);
% x2_plot.setXLabel('time [$s$]', 'interpreter','latex', 'fontsize',16);
% x2_plot.setTitle('UKF estimation results', 'interpreter','latex', 'fontsize',17);
% x2_plot.setLineProperties(1, 'LineWidth',2.0, 'Color','blue');
% x2_plot.setLineProperties(2, 'LineWidth',2.0, 'Color','magenta');

% kgain_plot = PlotLines1D(axes('Parent',figure), 2);
% kgain_plot.setLegend({'$K_1$','$K_2$'}, 'interpreter','latex', 'fontsize',15);
% kgain_plot.setYLabel('Kalman gain', 'interpreter','latex', 'fontsize',18);
% kgain_plot.setXLabel('time [$s$]', 'interpreter','latex', 'fontsize',16);
% kgain_plot.setTitle('UKF - Kalman gain', 'interpreter','latex', 'fontsize',17);
% kgain_plot.setLineProperties(1, 'LineWidth',2.0, 'Color','blue');
% kgain_plot.setLineProperties(2, 'LineWidth',2.0, 'Color','cyan');

for k=1:n_steps
    
    x_hat_data = [x_hat_data x_hat];
    P_data = [P_data P(:)];
    
    model.setExtParams(Time(k));
    
    % Residuals (or innovations): Measured output - Predicted output
    e(:,k) = y_data(:,k) - model.msrFun(ukf.theta); % ukf.State is x[k|k-1] at this point
    
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [theta_p, theta_m, Sigma_points] = ukf.correct(y_data(:,k));
    
    % kgain_plot.update(Time(k), ukf.K);
    % x1_plot.update(Time(k), [x_data(1,k); x_hat_data(1,k)]);
    % x2_plot.update(Time(k), [x_data(2,k); x_hat_data(2,k)]);

%     t = Time(k);
%     if (t >= 2.07)
%         pause;
%     else
%         pause(dt);
%     end
    
    
%     msr_up_plot.update(theta_p, theta_m, Sigma_points);
%     msr_up_plot.rescale();

    P = ukf.P;
    x_hat = ukf.theta;

    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    [theta_next, Sigma_points_next] = ukf.predict();
    
%     time_up_plot.update(x_hat, theta_next, Sigma_points_next);
%     time_up_plot.rescale();
  
    % t = Time(k)
    
%     pause(dt)
end

%% ========================================================================
%% ========================================================================

n_dim = size(x_data,1);

figure('Position',[282 128 1436 846]);
for i=1:n_dim
    subplot(n_dim,2,(i-1)*2+1);
    hold on;
    plot(Time,x_data(i,:), 'LineWidth',2.0, 'Color','blue');
    plot(Time,x_hat_data(i,:), 'LineWidth',2.0, 'Color','magenta');
    legend({['$x_' num2str(i) '$'],['$\hat{x}_' num2str(i) '$']}, 'interpreter','latex', 'fontsize',15);
    ylabel(['$x_' num2str(i) '$'], 'interpreter','latex', 'fontsize',17);
    if (i==1), title('UKF estimation results', 'interpreter','latex', 'fontsize',17); end
    hold off;
end

x_err_data = x_data-x_hat_data;
for i=1:n_dim
    subplot(n_dim,2,(i-1)*2+2);
    hold on;
    plot(Time,x_err_data(i,:), 'LineWidth',2.0, 'Color','blue');  % Error for the first state
    plot(Time,sqrt(P_data((i-1)*2+1,:)), 'LineWidth',2.0, 'Color','red');  % 1-sigma upper-bound
    plot(Time,-sqrt(P_data((i-1)*2+1,:)), 'LineWidth',2.0, 'Color','red');  % 1-sigma lower-bound
    xlabel('Time [$s$]', 'interpreter','latex', 'fontsize',15);
    ylabel(['$\tilde{x}_' num2str(i) '$'], 'interpreter','latex', 'fontsize',17);
    legend({'State estimate','$\pm \sigma$'}, 'interpreter','latex', 'fontsize',15);
    if (i==1), title('State estimation errors', 'interpreter','latex', 'fontsize',17); end
    hold off;
end



%% ========================================================================
%% ========================================================================
