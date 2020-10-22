clc;
% close all;
clear;

rng(0);

%% Time
dt = 0.0005;
tf = 5;
Time = 0:dt:tf;
n_steps = length(Time);

%% system
A = [0 1; -4 0];
C = [1/4 -3/4];
state_trans_fun = @(x) A*x;
msr_fun = @(x) C*x;
if (rank(obsv(A,C)) < size(A,2)), error('The system is not observable!'); end

%% cont KF params
R_ = 1;
Q_ = 0.1; % 10
P0 = 1;
a_p_cont = 0; % 0.99

%% create measurement noise
noise = sqrt(R_)*randn(1, n_steps);

%% logging
Time = zeros(1, n_steps);
x_data = zeros(2, n_steps);
y_data = zeros(1, n_steps);
y_msr_data = zeros(1, n_steps);

x_hat_data = zeros(2, n_steps);
y_hat_data = zeros(1, n_steps);

%% init state
t = 0;
x = [2; 1.5];
x_hat = [0; 0];
P = P0;

%% run simulation
sim_cont();
% sim_disc();

%% plot results
fig = figure;
fig.Position(3:4) = [1395 908];
k = 1;
for i=1:length(x)
    subplot(3,2,k); hold on;
    plot(Time, x_hat_data(i,:), 'LineWidth',2, 'Color','magenta');
    plot(Time, x_data(i,:), 'LineWidth',2, 'Color','blue', 'LineStyle','--');
    legend({['$x_' num2str(i) '$'], ['$\hat{x}_' num2str(i) '$']}, 'interpreter','latex', 'fontsize',15);
    subplot(3,2,k+1); hold on;
    plot(Time, x_data(i,:) - x_hat_data(i,:), 'LineWidth',2, 'Color','red');
    legend({['$x_' num2str(i) ' - \hat{x}_' num2str(i) '$']}, 'interpreter','latex', 'fontsize',15);
    k = k+2;
end
subplot(3,2,k); hold on;
plot(Time, y_msr_data, 'LineWidth',2, 'Color',[0 0 1 0.5], 'LineStyle','-');
plot(Time, y_hat_data, 'LineWidth',2, 'Color','magenta');
plot(Time, y_data, 'LineWidth',2, 'Color','green', 'LineStyle','--');
legend({'$y_{msr}$', '$\hat{y}$', '$y$'}, 'interpreter','latex', 'fontsize',15);
subplot(3,2,k+1); hold on;
plot(Time, y_data - y_hat_data, 'LineWidth',2, 'Color','red');
legend({'$y - \hat{y}$'}, 'interpreter','latex', 'fontsize',15);

