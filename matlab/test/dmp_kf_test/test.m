clc;
close all;
clear;

% Your initial state guess at time k, utilizing measurements up to time k-1: xhat[k|k-1]
initialStateGuess = [2;0]; % xhat[k|k-1]
% Construct the filter
ukf = unscentedKalmanFilter(@vdpStateFun, @vdpMsrFcn, initialStateGuess,...
    'HasAdditiveMeasurementNoise',true);

R = 0.2; % Variance of the measurement noise v[k]
ukf.MeasurementNoise = R;

ukf.ProcessNoise = diag([0.02 0.1]);

Ts = 0.05; % [s] Filter sample time
timeVector = 0:Ts:5;
[~,xTrue] = ode45(@vdp1,timeVector,[2;0]);

N = size(xTrue, 1);

rng(1); % Fix the random number generator for reproducible results
yTrue = zeros(1,N);
for j=1:N
    yTrue(:,j)
yMsr = yTrue + sqrt(R)*randn(size(yTrue)); % sqrt(R): Standard deviation of noise

Nsteps = numel(yMsr); % Number of time steps
xCorrectedUKF = zeros(Nsteps,2); % Corrected state estimates
PCorrected = zeros(Nsteps,2,2); % Corrected state estimation error covariances
e = zeros(Nsteps,1); % Residuals (or innovations)

for k=1:Nsteps
    % Let k denote the current time.
    %
    % Residuals (or innovations): Measured output - Predicted output
    e(k) = yMsr(k) - vdpMeasurementFcn(ukf.State); % ukf.State is x[k|k-1] at this point
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [xCorrectedUKF(k,:), PCorrected(k,:,:)] = ukf.correct(yMsr(k));
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    ukf.predict(Ts);
end

figure();
subplot(2,1,1);
plot(timeVector,xTrue(:,1),timeVector,xCorrectedUKF(:,1),timeVector,yMsr(:));
legend('True','UKF estimate','Measured')
ylim([-2.6 2.6]);
ylabel('x_1');
subplot(2,1,2);
plot(timeVector,xTrue(:,2),timeVector,xCorrectedUKF(:,2));
ylim([-3 1.5]);
xlabel('Time [s]');
ylabel('x_2');

figure();
plot(timeVector, e);
xlabel('Time [s]');
ylabel('Residual (or innovation)');

mean(e)

[xe,xeLags] = xcorr(e,'coeff'); % 'coeff': normalize by the value at zero lag
% Only plot non-negative lags
idx = xeLags>=0;
figure();
plot(xeLags(idx),xe(idx));
xlabel('Lags');
ylabel('Normalized correlation');
title('Autocorrelation of residuals (innovation)');

eStates = xTrue-xCorrectedUKF;
figure();
subplot(2,1,1);
plot(timeVector,eStates(:,1),...               % Error for the first state
    timeVector, sqrt(PCorrected(:,1,1)),'r', ... % 1-sigma upper-bound
    timeVector, -sqrt(PCorrected(:,1,1)),'r');   % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for state 1');
title('State estimation errors');
subplot(2,1,2);
plot(timeVector,eStates(:,2),...               % Error for the second state
    timeVector,sqrt(PCorrected(:,2,2)),'r', ...  % 1-sigma upper-bound
    timeVector,-sqrt(PCorrected(:,2,2)),'r');    % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for state 2');
legend('State estimate','1-sigma uncertainty bound',...
    'Location','Best');

distanceFromBound1 = abs(eStates(:,1))-sqrt(PCorrected(:,1,1));
percentageExceeded1 = nnz(distanceFromBound1>0) / numel(eStates(:,1));
distanceFromBound2 = abs(eStates(:,2))-sqrt(PCorrected(:,2,2));
percentageExceeded2 = nnz(distanceFromBound2>0) / numel(eStates(:,2));
[percentageExceeded1 percentageExceeded2]

mean(eStates)

[xeStates1,xeStatesLags1] = xcorr(eStates(:,1),'coeff'); % 'coeff': normalize by the value at zero lag
[xeStates2,xeStatesLags2] = xcorr(eStates(:,2),'coeff'); % 'coeff'
% Only plot non-negative lags
idx = xeStatesLags1>=0;
figure();
subplot(2,1,1);
plot(xeStatesLags1(idx),xeStates1(idx));
xlabel('Lags');
ylabel('For state 1');
title('Normalized autocorrelation of state estimation error');
subplot(2,1,2);
plot(xeStatesLags2(idx),xeStates2(idx));
xlabel('Lags');
ylabel('For state 2');


%% =======================================================================
%% =======================================================================

function yk = vdpMsrFcn(xk)
% Measurement function for discrete time nonlinear state estimators with 
% additive measurement noise.
% @param[in] xk: state at timestep k.
% @return: measurements at timestep k.
%

    if (nargin < 2), vk=0; end
    
    yk = xk(1);

end

function x = vdpStateFun(x, Ts) 
% Discrete-time approximation to van der Pol ODEs for mu = 1. 
% @param[in] x: state at timestep k.
% @param[in] Ts: sampling time in seconds. (optinal, default=0.05)
% @return: state at timestep k+1.
% 

    if (nargin < 2), Ts = 0.05; end

    % Euler integration of continuous-time dynamics x'=f(x) with sample time Ts
    x = x + vdpStateFunCont(x)*Ts;

end

function x_dot = vdpStateFunCont(x)
% Evaluate the van der Pol ODEs for mu = 1.
% @param[in] x: current state.
% @return: time derivative of the current state.
% 

    x_dot = [x(2); (1-x(1)^2)*x(2)-x(1)];

end