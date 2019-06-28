clc;
% close all;
clear;

global dt

dt = 0.005;

% Your initial state guess at time k, utilizing measurements up to time k-1: xhat[k|k-1]
x0_hat = [4; 0.6]; % xhat[k|k-1]
% Construct the filter
ekf = extendedKalmanFilter(@vdpStateTransFun, @vdpMsrFun, x0_hat);

Q = diag([0.02 0.1]); % Variance of the process noise w[k]
R = 0.2; % Variance of the measurement noise v[k]
P0 = 0.1*diag([10 10]);
ekf.MeasurementNoise = R;
ekf.ProcessNoise = Q;
ekf.StateCovariance = P0;
% ekf.StateTransitionJacobianFcn = @vdpStateTransFunJacob;
% ekf.MeasurementJacobianFcn = @vdpMsrFunJacob;

Time = 0:dt:5;
x_data = [];
x0 = [2;0];
x = x0;
for i=1:length(Time)
    x_data = [x_data x];
    dx = vdpStateTransFunCont(x);
    x = x + dx*dt;
end

rng(1); % Fix the random number generator for reproducible results
y_nn_data = vdpMsrFun(x_data);
y_data = y_nn_data .* (1+sqrt(R)*randn(size(y_nn_data))); % sqrt(R): Standard deviation of noise

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
    e(k) = y_data(k) - vdpMsrFun(ekf.State); % ukf.State is x[k|k-1] at this point
    
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


figure();
subplot(2,1,1);
hold on;
plot(Time,x_data(1,:), 'LineWidth',2.0, 'Color','blue');
plot(Time,x_hat_data(1,:), 'LineWidth',2.0, 'Color','magenta');
hold off;
legend('True','EKF estimate');
ylabel('x_1');
subplot(2,1,2);
hold on;
plot(Time,x_data(2,:), 'LineWidth',2.0, 'Color','blue');
plot(Time,x_hat_data(2,:), 'LineWidth',2.0, 'Color','magenta');
xlabel('Time [s]');
ylabel('x_2');
hold off;

% figure();
% plot(Time, e);
% xlabel('Time [s]');
% ylabel('Residual (or innovation)');

eStates = x_data-x_hat_data;
figure();
subplot(2,1,1);
plot(Time,eStates(1,:),...               % Error for the first state
    Time, sqrt(P_data(1,:)),'r', ... % 1-sigma upper-bound
    Time, -sqrt(P_data(1,:)),'r');   % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for state 1');
title('State estimation errors');
subplot(2,1,2);
plot(Time,eStates(2,:),...               % Error for the second state
    Time,sqrt(P_data(3,:)),'r', ...  % 1-sigma upper-bound
    Time,-sqrt(P_data(3,:)),'r');    % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for state 2');
legend('State estimate','1-sigma uncertainty bound',...
    'Location','Best');


%% ========================================================================
%% ========================================================================
%% ========================================================================
%% ========================================================================


function F = vdpStateTransFunJacob(x)

    global dt
    
    F = zeros(2,2);
    
    F(1,1) = 1;
    F(1,2) = dt;
    F(2,1) = -dt*(1+2*(1-x(1)));
    F(2,2) = 1 + x(2).*(1-x(1)).^2*dt;
    
end

function x = vdpStateTransFun(x) 
% @param[in] xk: States x[k]
% @param[out] xk1: Propagated states x[k+1]  
%

    global dt

    % Euler integration of continuous-time dynamics x'=f(x) with sample time dt
    x = x + vdpStateTransFunCont(x)*dt;
    
end


function dxdt = vdpStateTransFunCont(x)
% Evaluate the van der Pol ODEs for mu = 1

    dxdt = [x(2); (1-x(1)^2)*x(2)-x(1)];

end

function yk = vdpMsrFun(xk)
% @param[in] xk: x[k], states at time k
% @param[out] yk: y[k], measurements at time k
%
    %yk = 0.5*xk(1,:).^2 + xk(1,:).*xk(2,:) - 0.1*xk(2,:).^3;
    yk = xk(1,:).*xk(1,:);
    
end

function H = vdpMsrFunJacob(x)
% @param[in] xk: x[k], states at time k
% @param[out] yk: y[k], measurements at time k
%
    H = zeros(1,2);
    
    H(1) = 2*x(1);
    H(2) = 0;
    
end