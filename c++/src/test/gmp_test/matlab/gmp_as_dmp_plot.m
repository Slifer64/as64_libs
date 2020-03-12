addpath('utils/');

filename = 'data/gmp_as_dmp_sim_data.bin';

%% Load sim data
in = fopen(filename,'r');
if (in < 0), error(['Failed to load ''' filename '''']); end

Timed = read_mat(in, true);
Pd_data = read_mat(in, true);
dPd_data = read_mat(in, true);
ddPd_data = read_mat(in, true);

Time = read_mat(in, true);
P_data = read_mat(in, true);
dP_data = read_mat(in, true);
ddP_data = read_mat(in, true);
ks = read_scalar(in, true, 'double');
kt = read_scalar(in, true, 'double');

%% Plot results
figure;
subplot(3,1,1);
hold on;
plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
plot(Timed, Pd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
title(['temporal scale: $' num2str(kt) '$     ,     spatial scale: $' num2str(ks) '$'], 'interpreter','latex', 'fontsize',18);
legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);

axis tight;
hold off;

subplot(3,1,2);
hold on;
plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, dPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

subplot(3,1,3);
hold on;
plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, ddPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

%% ==================================================================
%% ==================================================================




