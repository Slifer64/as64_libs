clc;
close all;
clear;

k0 = 12;

getStiffness = @(t) k0 + 100*sin(t);

getXd = @(t) 10*sin(0.1*t);
getXdDot = @(t) cos(0.1*t);
getXdDDot = @(t) -0.1*sin(0.1*t);

M = 10; % mass
D = 30;

tf = 100;
dt = 0.01;

t = 0;
x = 0;
x_dot = 0;

Time = [];
x_data = [];
dx_data = [];
ddx_data = [];

xd_data = [];
dxd_data = [];
ddxd_data = [];

K_data = [];

while (t < tf)
    
    xd = getXd(t);
    xd_dot = getXdDot(t);
    xd_ddot = getXdDDot(t);
    
    K = getStiffness(t);
    
    x_ddot = xd_ddot + ( -D*(x_dot-xd_dot) -K*(x-xd) ) / M;
   
    Time = [Time t];

    x_data = [x_data x];
    dx_data = [dx_data x_dot];
    ddx_data = [ddx_data x_ddot];
    
    xd_data = [xd_data xd];
    dxd_data = [dxd_data xd_dot];
    ddxd_data = [ddxd_data xd_ddot];
    
    K_data = [K_data K];   
    
    t = t + dt;
    x = x + x_dot*dt;
    x_dot = x_dot + x_ddot*dt;
    
end


figure('Position',[611 52 560 893]);
subplot(4,1,1); hold on;
plot(Time, x_data, 'LineWidth',2, 'Color','blue', 'LineStyle','-');
plot(Time, xd_data, 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
subplot(4,1,2); hold on;
plot(Time, dx_data, 'LineWidth',2, 'Color','blue', 'LineStyle','-');
plot(Time, dxd_data, 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
subplot(4,1,3); hold on;
plot(Time, ddx_data, 'LineWidth',2, 'Color','blue', 'LineStyle','-');
plot(Time, ddxd_data, 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
subplot(4,1,4); hold on;
plot(Time, K_data, 'LineWidth',2, 'Color','red', 'LineStyle','-');
ylabel('K [$Nm/s$]', 'interpreter','latex', 'fontsize',15);


