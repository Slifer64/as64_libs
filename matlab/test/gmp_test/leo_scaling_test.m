clc;
close all;
clear;

rng(0);

%% Load training data
% load('data/train_data.mat', 'Data');
load('sdata.mat', 'Data');

Time = Data.Time;
Pd_data = Data.Pos;
dPd_data = Data.Vel;
ddPd_data = Data.Accel;

%% init
Pgd = Pd_data(:,end);
P0d = Pd_data(:,1);
P0 = P0d;
Pg = diag([1.1 1.08 1.9])*Pgd;
% Pg = P0 + diag([0.8; 0.7; 0.1])*(Pgd-P0);

%% proportional scaling
ks = diag( (Pg - P0) ./ (Pgd - P0d) );
P_data = ks*(Pd_data - P0d) + P0;
dP_data = ks*dPd_data;
ddP_data = ks*ddPd_data;


%% coord change scaling
nd = Pgd - P0d;  nd = nd/norm(nd);
n = Pg - P0;  n = n/norm(n);
k = cross(nd,n);
theta = acos(dot(n,nd));
S_k = vec2ssmat(k);
% R = eye(3,3) + sin(theta)*S_k + (1-cos(theta))*S_k*S_k;
R = axang2rotm([k' theta]);

ks_R = R*norm(Pg - P0)/norm(Pgd - P0d);
P_data2 = ks_R*(Pd_data - P0d) + P0;
dP_data2 = ks_R*dPd_data;
ddP_data2 = ks_R*ddPd_data;


%% Plot results
for i=1:3
    
    fig = figure;
    ax1 = subplot(3,1,1);
    hold on;
    plot(Time, P_data(i,:), 'LineWidth',2.0 , 'Color','blue', 'LineStyle','-');
    plot(Time, P_data2(i,:), 'LineWidth',2.0 , 'Color','magenta', 'LineStyle',':');
    plot(Time, Pd_data(i,:), 'LineWidth',2.0, 'Color','green', 'LineStyle','--');
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
    legend({'dmp','leo','demo'}, 'interpreter','latex', 'fontsize',15);
    axis tight;
    hold off;

    ax2 = subplot(3,1,2);
    hold on;
    plot(Time, dP_data(i,:), 'LineWidth',2.0 , 'Color','blue', 'LineStyle','-');
    plot(Time, dP_data2(i,:), 'LineWidth',2.0 , 'Color','magenta', 'LineStyle',':');
    plot(Time, dPd_data(i,:), 'LineWidth',2.0, 'Color','green', 'LineStyle','--');
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
    axis tight;
    hold off;

    ax3 = subplot(3,1,3);
    hold on;
    plot(Time, ddP_data(i,:), 'LineWidth',2.0 , 'Color','blue', 'LineStyle','-');
    plot(Time, ddP_data2(i,:), 'LineWidth',2.0 , 'Color','magenta', 'LineStyle',':');
    plot(Time, ddPd_data(i,:), 'LineWidth',2.0, 'Color','green', 'LineStyle','--');
    ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
    axis tight;
    hold off;
    
end

figure;
hold on;
plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth',2, 'Color','blue', 'LineStyle','-');
plot3(P_data2(1,:), P_data2(2,:), P_data2(3,:), 'LineWidth',2, 'Color','magenta', 'LineStyle',':');
plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth',2, 'Color','green', 'LineStyle','--');
xlabel('$x$-axis', 'interpreter','latex', 'fontsize',15);
ylabel('$y$-axis', 'interpreter','latex', 'fontsize',15);
zlabel('$z$-axis', 'interpreter','latex', 'fontsize',15);
legend({'dmp','leo','demo'}, 'interpreter','latex', 'fontsize',15);
hold off;


% Data.Time = Time;
% Data.Pos = P_data;
% Data.Vel = dP_data;
% Data.Accel = ddP_data;
% save('sdata.mat', 'Data');

%% ==================================================================
%% ==================================================================



function ssMat = vec2ssmat(v)

    ssMat(1,1) = 0;
    ssMat(2,2) = 0;
    ssMat(3,3) = 0;
    ssMat(1,2) = -v(3);
    ssMat(2,1) = v(3);
    ssMat(1,3) = v(2);
    ssMat(3,1) = -v(2);
    ssMat(3,2) = v(1);
    ssMat(2,3) = -v(1);

end
