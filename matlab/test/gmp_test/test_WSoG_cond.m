clc;
% close all;
clear;

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos(1,:);
dPd_data = Data.Vel(1,:);
ddPd_data = Data.Accel(1,:);

Ts = Timed(2)-Timed(1);


T = Timed(end);
Time = Timed;
x = Time / T;
dx = 1/T;
ddx = 0;

%% initialize and train GMP
N_kernels = 50;
wsog = WSoG(N_kernels, 2);
tic
offline_train_mse = wsog.train(DMP_TRAIN.LS, x, Pd_data);
offline_train_mse
toc

% wsog.plotPsi(x);

N = length(x);
P_data = zeros(1,N);
dP_data = zeros(1,N);
ddP_data = zeros(1,N);

t1 = 2;
x1 = t1/T;
p1 = 0.3;
% wsog.updatePos(x1, p1);

t2 = 3;
x2 = t2/T;
dx2 = 1/T;
p2_dot = 0.1;
% wsog.updateVel(x2, dx2, p2_dot);

t3 = 4.25;
x3 = t3/T;
dx3 = 1/T;
p3_ddot = 0.3;
% wsog.updateAccel(x3, dx3, p3_ddot);


t4 = 8;
x4 = t4/T;
dx4 = 1/T;
p4 = 0.4;
p4_dot = 0.2;
p4_ddot = 0.0;
wsog.updatePosVelAccel(x4, dx4, p4, p4_dot, p4_ddot);

for i=1:N
    P_data(i) = wsog.output(x(i));
    dP_data(i) = wsog.outputDot(x(i), dx);
    ddP_data(i) = wsog.outputDDot(x(i), dx, ddx);
end


%% Plot results
figure;
subplot(3,1,1);
hold on;
plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
plot(Timed, Pd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
scatter([t1], [p1], 'MarkerEdgeColor','red', 'LineWidth',2, 'SizeData', 100);
plot([t1 t1], ylim, 'LineWidth',1, 'Color','green', 'LineStyle','--');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);

axis tight;
hold off;

subplot(3,1,2);
hold on;
plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, dPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
scatter([t2], [p2_dot], 'MarkerEdgeColor','red', 'LineWidth',2, 'SizeData', 100);
plot([t2 t2], ylim, 'LineWidth',1, 'Color','green', 'LineStyle','--');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

subplot(3,1,3);
hold on;
plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, ddPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
scatter([t3], [p3_ddot], 'MarkerEdgeColor','red', 'LineWidth',2, 'SizeData', 100);
plot([t3 t3], ylim, 'LineWidth',1, 'Color','green', 'LineStyle','--');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;







