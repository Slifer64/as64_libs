clc;
close all;
clear;

set_matlab_utils_path();

%% Load training data

path = strrep(mfilename('fullpath'), 'quat_integr_test','');

load([path '/quat_integr_test/data/train_data3.mat'], 'Data');
% load('data/orient_data.mat', 'Data');

Time = Data.Time;
Qd_data = Data.Quat;
vRotd_data = Data.RotVel;
dvRotd_data = Data.RotAccel;

Q0 = Qd_data(:,1);
dt = Time(2) - Time(1);

%% Integrate Q_dot
Q_data = zeros(size(Qd_data));
Q = Q0;
for i=1:length(Time)
    Q_data(:,i) = Q;
    vRot = vRotd_data(:,i);
    dQ = 0.5*quatProd([0; vRot],Q);
    Q = Q + dQ*dt;
    Q = Q/norm(Q);
end


figure;
for i=1:4
    subplot(4,1,i);
    hold on;
    plot(Time, Qd_data(i,:), 'LineWidth',2);
    plot(Time,Q_data(i,:), 'LineWidth',2);
    if (i==1), legend({'$Q_d$','$Q$'}, 'interpreter','latex', 'fontsize',15); end
    if (i==1), title('Integration with $\dot{Q}$', 'interpreter','latex', 'fontsize',15); end
    if (i==4), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
    hold off;
end

%% Integrate dot(k*theta)
Q_data = zeros(size(Qd_data));
eo = quatLog(Q0);
for i=1:length(Time)
    Q = quatExp(eo);
    Q_data(:,i) = Q;
    vRot = vRotd_data(:,i);
    
    J_deo_dQ = DMP_eo.jacobDeoDquat(Q);
    deo = 0.5*J_deo_dQ * quatProd([0; vRot],Q);
    eo = eo + deo*dt;
end


figure;
for i=1:4
    subplot(4,1,i);
    hold on;
    plot(Time, Qd_data(i,:), 'LineWidth',2);
    plot(Time,Q_data(i,:), 'LineWidth',2);
    if (i==1), legend({'$Q_d$','$Q$'}, 'interpreter','latex', 'fontsize',15); end
    if (i==1), title('Integration with $\frac{d}{dt}(\mathbf{k}\theta)$', 'interpreter','latex', 'fontsize',15); end
    if (i==4), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
    hold off;
end

%% Integrate dot(k*theta) with equation from Dora paper
Q_data = zeros(size(Qd_data));
eo = quatLog(Q0);
for i=1:length(Time)
    Q = quatExp(eo);
    Q_data(:,i) = Q;
    vRot = vRotd_data(:,i);
    
    r = norm(eo);
    deo = 0.5*cross(vRot,eo) + vRot*r*cot(r)/2 + eo*dot(eo,vRot)*(1-r*cot(r))/(2*r^2);
    eo = eo + deo*dt;
end

figure;
for i=1:4
    subplot(4,1,i);
    hold on;
    plot(Time, Qd_data(i,:), 'LineWidth',2);
    plot(Time,Q_data(i,:), 'LineWidth',2);
    if (i==1), legend({'$Q_d$','$Q$'}, 'interpreter','latex', 'fontsize',15); end
    if (i==1), title('Integration with $\frac{d}{dt}(\mathbf{k}\theta)$ from Dora paper', 'interpreter','latex', 'fontsize',15); end
    if (i==4), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
    hold off;
end



