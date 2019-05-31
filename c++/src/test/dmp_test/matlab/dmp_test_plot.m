
addpath('utils/');

filename = 'data/dmp_sim_data.bin';
%% Load sim data
in = fopen(filename,'r');
if (in < 0), error(['Failed to load ''' filename '''']); end

Timed = read_mat(in, true);
yd_data = read_mat(in, true);
dyd_data = read_mat(in, true);
ddyd_data = read_mat(in, true);
Time = read_mat(in, true);
y_data = read_mat(in, true);
dy_data = read_mat(in, true);
ddy_data = read_mat(in, true);

fclose(in);

%% Plot results

figure;
subplot(3,1,1);
hold on;
plot(Time, y_data, 'LineWidth',2.0 , 'Color','blue');
plot(Timed, yd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

subplot(3,1,2);
hold on;
plot(Time, dy_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, dyd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

subplot(3,1,3);
hold on;
plot(Time, ddy_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, ddyd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

