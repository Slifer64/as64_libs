function trainDMP()

set_matlab_utils_path();

%% Load training data

path = strrep(mfilename('fullpath'), 'trainDMP','');

load([path 'data/train_data2.mat'], 'Data');

Timed = Data.Time;
Pd_data = Data.Pos;
dPd_data = Data.Vel;
ddPd_data = Data.Accel;
Qd_data = Data.Quat;
vRotd_data = Data.RotVel;
dvRotd_data = Data.RotAccel;

Ts = Timed(2)-Timed(1);

Pd_data = Pd_data-Pd_data(:,end);
Qgd = Qd_data(:,end);
for j=1:length(Timed)
    Qd_data(:,j) = quatProd(Qd_data(:,j), quatInv(Qgd));
end

%% initialize DMP
a_z = 20;
b_z = a_z/4;
train_method = DMP_TRAIN.LWR;
can_clock_ptr = CanonicalClock();
shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
% shape_attr_gat_ptr = LinGatingFunction(1.0, 0.01);
N_kernels = [40; 40; 40];
dmp_p = DMP_pos(DMP_TYPE.STD, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);
dmp_o = DMP_eo(DMP_TYPE.STD, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);

disp('DMP pos training...')
tic
offline_train_mse = dmp_p.train(train_method, Timed, Pd_data, dPd_data, ddPd_data);
offline_train_mse
toc

disp('DMP orient training...')
tic
offline_train_mse = dmp_o.train(train_method, Timed, Qd_data, vRotd_data, dvRotd_data);
offline_train_mse
toc

%% DMP simulation
disp('DMP simulation...');
tic
P0 = Pd_data(:,1);
Pg = Pd_data(:,end);
Q0 = Qd_data(:,1);
Qgd = Qd_data(:,end);
ks = 1.0;
e0 = ks*quatLog( quatProd( Qgd, quatInv(Q0) ) );
Qg = quatProd(quatExp(e0), Q0); %quatExp(1.0*quatLog(Qd_data(:,end)));

% Qg_offset = [0.2 0.7 0.8 0.0]';
% Qg_offset = Qg_offset/norm(Qg_offset);
% Qg = quatProd(Qg_offset, Qgd);

T = 1.0*Timed(end);
dt = Ts;
[Time, P_data, dP_data, ddP_data, Q_data, vRot_data, dvRot_data] = simulatePosOrientDMP(dmp_p, dmp_o, P0, Q0, Pg, Qg, T, dt);
toc

% Data.Time = Time;
% Data.Pos = P_data;
% Data.Vel = dP_data;
% Data.Accel = ddP_data;
% Data.Quat = Q_data;
% Data.RotVel = vRot_data;
% Data.RotAccel = dvRot_data;
% save('data/train_data2.mat', 'Data');
% return

Yg0 = Pd_data(:,end);
Y0 = Pd_data(:,1);
Qg0 = Qd_data(:,end);
Q0 = Qd_data(:,1);
tau0 = Timed(end);
save([path 'data/dmp_data.mat'],'dmp_p','dmp_o', 'Yg0', 'Y0', 'Qg0', 'Q0', 'tau0');

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

% ============= Pos-Orient path ================
figure;
ax = plot_3Dpath_with_orientFrames(Pd_data, Qd_data, 'LineWidth',3.0, 'LineStyle','-', 'numberOfFrames',12, 'frameScale',0.2, 'frameLineWidth',2.0, ...
    'FrameXAxisLegend','$x_d$-axis', 'FrameYAxisLegend','$y_d$-axis', 'FrameZAxisLegend','$z_d$-axis', 'LineLegend','demo-path', 'LegendFontSize',14);
    
plot_3Dpath_with_orientFrames(P_data(:,1:4:end), Q_data(:,1:4:end), 'axes',ax, 'LineWidth',2.0, 'LineStyle','--', 'LineColor',[0.93 0.69 0.13], 'numberOfFrames',12, 'frameScale',0.2, 'frameLineWidth',2.0, ...
    'frameXAxisColor', [1 0 1], 'frameYAxisColor', [0 0.5 0], 'frameZAxisColor', [0 1 1], 'frameLineStyle','--', 'animated',false, ...
    'FrameXAxisLegend','$x$-axis', 'FrameYAxisLegend','$y$-axis', 'FrameZAxisLegend','$z$-axis', 'LineLegend','sim-path', 'LegendFontSize',14);

end


