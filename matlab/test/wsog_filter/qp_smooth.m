clc;
close all;
clear;

set_matlab_utils_path();

load('data/train_data.mat', 'Data');

sps = 1;
n1 = 1;
n_step = 5;
n2 = length(Data.Time);
Timed = Data.Time(n1:n_step:n2);
Pd_data0 = sps*Data.Pos(1,n1:n_step:n2);
dPd_data0 = sps*Data.Vel(1,n1:n_step:n2);
ddPd_data0 = sps*Data.Accel(1,n1:n_step:n2);

Ts = Timed(2) - Timed(1);

Pd_data = Pd_data0 + sps*0.01*rand(size(Pd_data0));

dPd_data = zeros(size(Pd_data));
ddPd_data = zeros(size(Pd_data));
d3Pd_data = zeros(size(Pd_data));
for i=1:size(d3Pd_data,1)
    dPd_data(i,:) = [diff(Pd_data(i,:)) 0]/Ts; 
    ddPd_data(i,:) = [diff(dPd_data(i,:)) 0]/Ts; 
    d3Pd_data(i,:) = [diff(ddPd_data(i,:)) 0]/Ts; 
end


%% ============================================
%  L2 smoothness
%  min || x - x_hat ||_2 + a1*|| D*x_hat ||_2
% n = length(Pd_data);
% Phi = 1.0*eye(n,n);
% D = -eye(n-1,n);
% for i=1:n-1, D(i,i+1)=1; end
% a1 = 250;
% H = Phi + a1*D'*D;
% f = -Pd_data*Phi;
% P_data = quadprog(H,f)';

%% ============================================
%  L1 smoothness ??
%  min || x - x_hat ||_2 + a1*|| D*x_hat ||_1 + 0.01*||t||_2
%  The last term is added to make the Hessian positive definite. Besides 
%  the minimum will be the same, either ||t||_2 is added or not!
n = length(Pd_data);
n2 = n-1;
N = n + n2;
Phi = zeros(N, N);
Phi(1:n, 1:n) = 1.0*eye(n,n);
Phi(n+1:N, n+1:N) = 0.01*eye(n2,n2);

D = -eye(n-1,n);
for i=1:n-1, D(i,i+1)=1; end

o1 = ones(n2, 1);
I1 = ones(n2, n2);

a1 = 200;
H = Phi;
f = [-Pd_data a1*o1'];
A = [[D -I1]; [-D -I1]];
b = zeros(2*n2,1);
P_data = quadprog(H,f,A,b)';
P_data = P_data(1:n);



%% ============================================
%  Reconstruct velocity, acceleration
dP_data = zeros(size(P_data));
ddP_data = zeros(size(P_data));
for i=1:size(P_data,1)
    dP_data(i,:) = [diff(P_data(i,:)) 0]/Ts; 
    ddP_data(i,:) = [diff(dP_data(i,:)) 0]/Ts; 
end

%% =====================================================
%% =====================================================

% plotDemoSim(Time, P_data, dP_data, ddP_data, Timed, Pd_data, dPd_data, ddPd_data);
plotDemoSim(Timed, P_data, dP_data, ddP_data, Timed, Pd_data0, dPd_data0, ddPd_data0);


%% =====================================================
%% =====================================================



function plotDemoSim(Time, P_data, dP_data, ddP_data, Timed, Pd_data, dPd_data, ddPd_data)

    figure;
    subplot(3,1,1);
    hold on;
    plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
    plot(Timed, Pd_data(1,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
    legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);
    axis tight;
    min_y = min([Pd_data(1,:) P_data(1,:)]);
    max_y = max([Pd_data(1,:) P_data(1,:)]);
    y_off = (max_y - min_y)*0.1;
    ylim([min_y-y_off max_y+y_off]);
    hold off;


    subplot(3,1,2);
    hold on;
    plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
    plot(Timed, dPd_data(1,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
    axis tight;
    min_y = min([dPd_data(1,:) dP_data(1,:)]);
    max_y = max([dPd_data(1,:) dP_data(1,:)]);
    y_off = (max_y - min_y)*0.1;
    ylim([min_y-y_off max_y+y_off]);
    hold off;

    subplot(3,1,3);
    hold on;
    plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
    plot(Timed, ddPd_data(1,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
    axis tight;
    min_y = min([ddPd_data(1,:) ddP_data(1,:)]);
    max_y = max([ddPd_data(1,:) ddP_data(1,:)]);
    y_off = (max_y - min_y)*0.1;
    ylim([min_y-y_off max_y+y_off]);
    hold off;
    
end