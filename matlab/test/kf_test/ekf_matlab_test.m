clc;
% close all;
clear;

set_matlab_utils_path();

dt = 0.005;

vdp = VDP(dt);

% Your initial state guess at time k, utilizing measurements up to time k-1: xhat[k|k-1]
x0_hat = [4; 0.8]; % xhat[k|k-1]
Q = 1*diag([0.01 0.01]); % Variance of the process noise w[k]
R = 0.2; % Variance of the measurement noise v[k]
R_hat = 10*R;
P0 = 10*diag([1 1]);

%% Construct the filter
ekf = extendedKalmanFilter(@vdp.stateTransFun, @vdp.msrFun, x0_hat);
ekf.MeasurementNoise = R_hat;
ekf.ProcessNoise = Q;
ekf.StateCovariance = P0;
% ekf.StateTransitionJacobianFcn = @vdp.stateTransFunJacob;
% ekf.MeasurementJacobianFcn = @vdp.msrFunJacob;

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
P = ekf.StateCovariance;
x_hat = ekf.State;

for k=1:n_steps
    
    x_hat_data = [x_hat_data x_hat];
    P_data = [P_data P(:)];
    
    % Residuals (or innovations): Measured output - Predicted output
    e(k) = y_data(k) - vdp.msrFun(ekf.State); % ukf.State is x[k|k-1] at this point
    
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    ekf.correct(y_data(k));
    
    P = ekf.StateCovariance;
    x_hat = ekf.State;

    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    ekf.predict();
end


%% ========================================================================
%% ========================================================================

figure('Position',[282 128 1436 846]);
subplot(2,2,1);
hold on;
plot(Time,x_data(1,:), 'LineWidth',2.0, 'Color','blue');
plot(Time,x_hat_data(1,:), 'LineWidth',2.0, 'Color','magenta');
legend({'$x_1$','$\hat{x}_1$'}, 'interpreter','latex', 'fontsize',15);
ylabel('$x_1$', 'interpreter','latex', 'fontsize',17);
title('EKF matlab estimation results', 'interpreter','latex', 'fontsize',17);
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