clc;
% close all;
clear;

set_matlab_utils_path();

setParams();

% Construct the filter
ukf = unscentedKalmanFilter(@model.stateTransFun, @model.msrFun, x0_hat, 'HasAdditiveMeasurementNoise',true);
ukf.Alpha = 1e-3;
ukf.Beta = 2;
ukf.Kappa = 0;
ukf.StateCovariance = P0;
ukf.MeasurementNoise = R_hat;
ukf.ProcessNoise = Q;

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
P = ukf.StateCovariance;
x_hat = ukf.State;

for k=1:n_steps
    
    x_hat_data = [x_hat_data x_hat];
    P_data = [P_data P(:)];
    
    model.setExtParams(Time(k));
    
    % Residuals (or innovations): Measured output - Predicted output
    e(:,k) = y_data(:,k) - model.msrFun(ukf.State); % ukf.State is x[k|k-1] at this point
    
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    ukf.correct(y_data(:,k));
    
    P = ukf.StateCovariance;
    x_hat = ukf.State;

    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    ukf.predict();
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
    if (i==1), title('UKF matlab estimation results', 'interpreter','latex', 'fontsize',17); end
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
