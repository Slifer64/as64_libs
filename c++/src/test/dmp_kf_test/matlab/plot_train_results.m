
path = strrep(mfilename('fullpath'), 'plot_train_results','');
addpath([path '/utils/']);

%% Load train data
filename = [path '/data/train/train_data.bin'];
in = fopen(filename,'r');
if (in < 0), error(['Failed to load ''' filename '''']); end

Timed = read_mat(in, true);
Pd_data = read_mat(in, true);
dPd_data = read_mat(in, true);
ddPd_data = read_mat(in, true);
Qd_data = read_mat(in, true);
vRotd_data = read_mat(in, true);
dvRotd_data = read_mat(in, true);

fclose(in);

%% Load sim data
filename = [path '/data/sim/sim_data.bin'];
in = fopen(filename,'r');
if (in < 0), error(['Failed to load ''' filename '''']); end

Time = read_mat(in, true);
P_data = read_mat(in, true);
dP_data = read_mat(in, true);
ddP_data = read_mat(in, true);
Q_data = read_mat(in, true);
vRot_data = read_mat(in, true);
dvRot_data = read_mat(in, true);
Qg = read_mat(in, true);

fclose(in);

%% Plot results

fontsize = 14;

% ============= Position ================
figure;
titles = {'$X$', '$Y$', '$Z$'};
for i=1:3
    subplot(3,3,i);
    hold on;
    plot(Time, P_data(i,:), 'LineWidth',2.0, 'Color','blue');
    plot(Timed, Pd_data(i,:), 'LineWidth',2.0, 'Color','magenta', 'LineStyle','--');
    if (i==1), ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',fontsize); end
    if (i==3), legend({'sim','demo'}, 'interpreter','latex', 'fontsize',fontsize); end
    title(titles{i}, 'Interpreter','latex', 'fontsize',fontsize);
    axis tight;
    hold off;
end
for i=1:3
    subplot(3,3,3+i);
    hold on;
    plot(Time, dP_data(i,:), 'LineWidth',2.0, 'Color','blue');
    plot(Timed, dPd_data(i,:), 'LineWidth',2.0, 'Color','magenta', 'LineStyle','--');
    if (i==1), ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',fontsize); end
    axis tight;
    hold off;
end
for i=1:3
    subplot(3,3,6+i);
    hold on;
    plot(Time, ddP_data(i,:), 'LineWidth',2.0, 'Color','blue');
    plot(Timed, ddPd_data(i,:), 'LineWidth',2.0, 'Color','magenta', 'LineStyle','--');
    if (i==1), ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',fontsize); end
    axis tight;
    hold off;
end

% ============= Orientation ================
Eod_data = zeros(3, length(Timed));
for j=1:length(Timed), Eod_data(:,j) = quatLog( quatProd(Qd_data(:,end), quatInv(Qd_data(:,j))) ); end
Eo_data = zeros(3, length(Time));
for j=1:length(Time), Eo_data(:,j) = quatLog( quatProd(Qg, quatInv(Q_data(:,j))) ); end

figure;
titles = {'$X$', '$Y$', '$Z$'};
for i=1:3
    subplot(3,3,i);
    hold on;
    plot(Time, Eo_data(i,:), 'LineWidth',2.0, 'Color','blue');
    plot(Timed, Eod_data(i,:), 'LineWidth',2.0, 'Color','magenta', 'LineStyle','--');
    if (i==1), ylabel('pos [$rad$]', 'interpreter','latex', 'fontsize',fontsize); end
    if (i==3), legend({'sim','demo'}, 'interpreter','latex', 'fontsize',fontsize); end
    title(titles{i}, 'Interpreter','latex', 'fontsize',fontsize);
    axis tight;
    hold off;
end
for i=1:3
    subplot(3,3,3+i);
    hold on;
    plot(Time, vRot_data(i,:), 'LineWidth',2.0, 'Color','blue');
    plot(Timed, vRotd_data(i,:), 'LineWidth',2.0, 'Color','magenta', 'LineStyle','--');
    if (i==1), ylabel('vel [$rad/s$]', 'interpreter','latex', 'fontsize',fontsize); end
    axis tight;
    hold off;
end
for i=1:3
    subplot(3,3,6+i);
    hold on;
    plot(Time, dvRot_data(i,:), 'LineWidth',2.0, 'Color','blue');
    plot(Timed, dvRotd_data(i,:), 'LineWidth',2.0, 'Color','magenta', 'LineStyle','--');
    if (i==1), ylabel('accel [$rad/s^2$]', 'interpreter','latex', 'fontsize',fontsize); end
    axis tight;
    hold off;
end
