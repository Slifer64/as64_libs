clc;
close all;
clear;

set_matlab_utils_path();

% m = 16; 
% n = 8;
% A = randn(m,n);
% b = randn(m,1);
% 
% cvx_begin
%     variable x(n)
%     minimize( norm(A*x-b) )
% cvx_end
% 
% return 


load('data/train_data.mat', 'Data');

sps = 1;
n1 = 1;
n_step = 2;
n2 = length(Data.Time);
Timed = Data.Time(n1:n_step:n2);
Pd_data0 = sps*Data.Pos(1,n1:n_step:n2);
dPd_data0 = sps*Data.Vel(1,n1:n_step:n2);
ddPd_data0 = sps*Data.Accel(1,n1:n_step:n2);

Ts = Timed(2) - Timed(1);
Pd_data = Pd_data0 + sps*0.01*rand(size(Pd_data0));
dPd_data = [diff(Pd_data) 0]/Ts; 
ddPd_data = [diff(dPd_data) 0]/Ts;



%% =========================================
n = length(Pd_data);
xd = Pd_data';
D = -eye(n-1,n);
for i=1:n-1, D(i,i+1)=1; end
a1 = 20;

cvx_begin

    variable x(n)
    expressions J e

    e = x - xd;
    J = dot(e,e) + a1*norm(D*x,2);

    minimize( J )

cvx_end

%% =========================================

P_data = x';
dP_data = [diff(P_data) 0]/Ts; 
ddP_data = [diff(dP_data) 0]/Ts;

plotDemoSim(Timed, P_data, dP_data, ddP_data, Timed, Pd_data, dPd_data, ddPd_data);
plotDemoSim(Timed, P_data, dP_data, ddP_data, Timed, Pd_data0, dPd_data0, ddPd_data0);


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
