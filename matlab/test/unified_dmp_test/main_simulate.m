clc;
close all;
clear;

load('data.mat', 'Time','Pd_data','smp');

tau = Time(end);
x = Time / tau;
n_data = length(Time);

dx = 1/tau;
[P_data, dP_data, ddP_data] = smp.simulate(x, dx, 0);


P0 = Pd_data(:,1);
Pg = Pd_data(:,end);

ks = 1.5;
Pg2 = ks*(Pg - P0) + P0;

% calc scaled demo
Pd2_data = ks*(Pd_data - P0) + P0;
% dPd2_data = ks*dPd_data;
% ddPd2_data = ks*ddPd_data;

% update weights based on intermediate position
k = round(n_data*0.575);
xk = x(k);
Pk = Pd2_data(:,k);
% x_m = [x0; xk];
% z_m = [P0; Pk];
x_m = [xk];
z_m = [Pk];

% calc scaled MP
smp.updatePos(x_m, z_m); 
[P2_data, dP2_data, ddP2_data] = smp.simulate(x, dx, 0);

figure;
subplot(2,1,1); hold on;
plot(x, P2_data, 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
plot(x, Pd2_data, 'LineWidth',2, 'LineStyle','--', 'Color','blue');
plot(x, Pd_data, 'LineWidth',2, 'LineStyle','--', 'Color','green');
scatter(xk, Pk, 'SizeData',150, 'LineWidth',5, 'Marker','o', 'MarkerEdgeColor','red');
legend({'MP','DMP','demo','$P_k$'}, 'interpreter','latex', 'fontsize',15);
axis tight;
ax = subplot(2,1,2); hold on;
smp.plotPsi(x, ax);
xlabel('canonical time [$t/\tau$]', 'interpreter','latex', 'fontsize',15);


