clc;
close all;
clear;

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
yd_data = Data.Pos(1,:);
dyd_data = Data.Vel(1,:);
ddyd_data = Data.Accel(1,:);

Ts = Timed(2)-Timed(1);

%% initialize and train GMP
train_method = 'LWR';
N_kernels = 80;
kernels_std_scaling = 2;
wsog = WSoG(N_kernels, kernels_std_scaling);
tic
xd_data = Timed/Timed(end);
% yd_data = [ones(1,50)*yd_data(1) yd_data ones(1,50)*yd_data(end)];
% xd_data = (0:(length(yd_data)-1))/length(yd_data);
offline_train_mse = wsog.train(train_method, xd_data, yd_data);
offline_train_mse
toc

%% Compare with the learned trajectory
% [Timed, Pd_data, dPd_data, ddPd_data] = simulateGMP_nDoF(gmp, Pd_data(:,1), Pd_data(:,end), Timed(end), Ts);

%% DMP simulation
disp('WSoG simulation...');

y0 = yd_data(:,1);
yg = yd_data(:,end);
wsog.setStartValue(y0);
wsog.setFinalValue(yg);

tau = Timed(end);
dt = 0.01;
x_dot = 1/tau;
x_ddot = 0;

%% set initial values
x = -1.5;

x_data = [];
y_data = [];
dy_data = [];
ddy_data = [];
phi_data = [];

%% simulate
while (true)

    %% WSoG update
    y = wsog.output(x);
    y_dot = wsog.outputDot(x, x_dot);
    y_ddot = wsog.outputDDot(x, x_dot, x_ddot);
    
    phi = wsog.kernelFun(x);
    
    
    %% data logging
    x_data = [x_data x];
    y_data = [y_data y];
    dy_data = [dy_data y_dot];  
    ddy_data = [ddy_data y_ddot];
    phi_data = [phi_data phi];

    %% Stopping criteria
    if (x>=2.5), break; end

    %% Numerical integration
    x = x + x_dot*dt;
    
end

tol = 1e-10;
n_data = size(phi_data,2);
j1 = 1;
j2 = n_data;
for j=1:n_data
    if (phi_data(1,j) >= tol)
        j1 = j;
        break;
    end
end
for j=n_data:-1:j1
    if (phi_data(end,j) >= tol)
        j2 = j;
        break;
    end
end


n_sp = 4;
figure;
subplot(n_sp, 1, 1);
hold on;
plot(x_data, y_data, 'LineWidth',2, 'Color','blue', 'LineStyle','-');
plot(xd_data, yd_data, 'LineWidth',2, 'Color',[0.85 0.33 0.1 0.8], 'LineStyle','-.');
plot([x_data(1) x_data(end)], [yd_data(end) yd_data(end)], 'LineWidth',2, 'Color',[1 0 0 0.5], 'LineStyle','--');
axis tight;
hold off;
subplot(n_sp, 1, 2);
hold on;
plot(x_data, dy_data, 'LineWidth',2, 'Color','blue', 'LineStyle','-');
% plot(xd_data, dyd_data, 'LineWidth',2, 'Color',[0.85 0.33 0.1 0.8], 'LineStyle','-.');
plot([x_data(1) x_data(end)], [0 0], 'LineWidth',2, 'Color',[1 0 0 0.5], 'LineStyle','--');
axis tight;
ylim(1.1*[min(dyd_data) max(dyd_data)])
hold off;
subplot(n_sp, 1, 3);
hold on;
plot(x_data, ddy_data, 'LineWidth',2, 'Color','blue', 'LineStyle','-');
% plot(xd_data, ddyd_data, 'LineWidth',2, 'Color',[0.85 0.33 0.1 0.8], 'LineStyle','-.');
plot([x_data(1) x_data(end)], [0 0], 'LineWidth',2, 'Color',[1 0 0 0.5], 'LineStyle','--');
axis tight;
ylim(1.1*[min(ddyd_data) max(ddyd_data)])
hold off;

subplot(n_sp, 1, 4);
hold on;
for i=1:size(phi_data,1), plot(x_data, phi_data(i,:), 'LineWidth',2); end
plot([x_data(j1) x_data(j1)], ylim, 'LineWidth',2, 'Color',[1 0 0], 'LineStyle','--');
plot([x_data(j2) x_data(j2)], ylim, 'LineWidth',2, 'Color',[1 0 0], 'LineStyle','--');
axis tight;
hold off;





