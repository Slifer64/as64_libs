function dmp_eo_test2()

format compact;

set_matlab_utils_path();

%% Load training data

load('data/train_data1.mat', 'Data');
% load('data/orient_data.mat', 'Data');

Timed = Data.Time;
% Pd_data = Data.Pos;
% dPd_data = Data.Vel;
% ddPd_data = Data.Accel;
Qd_data = Data.Quat;
vRotd_data = Data.RotVel;
dvRotd_data = Data.RotAccel;

%% Write data to binary format
% fid = fopen('train_data.bin','w');
% write_mat(Timed, fid, true);
% write_mat(Qd_data, fid, true);
% write_mat(vRotd_data, fid, true);
% write_mat(dvRotd_data, fid, true);
% fclose(fid);

Ts = Timed(2)-Timed(1);

%% initialize DMP
a_z = 20;
b_z = a_z/4;
train_method = DMP_TRAIN.LS;
can_clock_ptr = CanonicalClock();
% shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
shape_attr_gat_ptr = LinGatingFunction(1.0, 0.01);
N_kernels = [40; 40; 40];
dmp_o = DMP_eo(DMP_TYPE.STD, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);

disp('DMP eo training...')
tic
offline_train_mse = dmp_o.train(train_method, Timed, Qd_data, vRotd_data, dvRotd_data);
offline_train_mse
toc

%% DMP simulation
disp('DMP simulation...');
tic
Q0 = Qd_data(:,1);
Qgd = Qd_data(:,end);
ks = 1.0;
e0 = ks*quatLog( quatProd( Qgd, quatInv(Q0) ) );
Qg = quatProd(quatExp(e0), Q0); %quatExp(1.0*quatLog(Qd_data(:,end)));
T = 1.0*Timed(end);
dt = Ts;

% Qg_offset = [0.2 0.7 0.8 0.0]';
% Qg_offset = Qg_offset/norm(Qg_offset);
Qg_offset = rotm2quat(rotx(20))';
Qg = quatProd(Qg_offset, Qgd);

ks = quatLog( quatProd( Qg, quatInv(Q0) ) ) ./ quatLog( quatProd( Qgd, quatInv(Q0) ) );
ks

[Time, Q_data, vRot_data, dvRot_data] = simulateDMPeo_in_eo_space(dmp_o, Q0, Qg, T, dt);
toc

%% Plot results

Q_ref = Q0;

Qgd = Qd_data(:,end);
Pqd_data = zeros(3, size(Qd_data,2));
for j=1:size(Pqd_data,2)
    Pqd_data(:,j) = quatLog( quatProd(Qgd, quatInv(Qd_data(:,j))) );
end

Pq_data = zeros(3, size(Q_data,2));
for j=1:size(Pq_data,2)
    Pq_data(:,j) = quatLog( quatProd(Qg, quatInv(Q_data(:,j))) );
end

line_width = 2.5;
 
figure('Position', [200 200 600 500]);
y_labels = {'$e_{q,x}$','$e_{q,y}$', '$e_{q,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, Pq_data(i,:), 'LineWidth', line_width);
   plot(Timed, ks(i)*Pqd_data(i,:), 'LineWidth', line_width, 'LineStyle','--');
   ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',20);
   axis tight;
   if (i==1), legend({'new', 'ks*demo'}, 'interpreter','latex', 'fontsize',16, 'Position',[0.7 0.78 0.27 0.15]); end
   if (i==1), title(['Quaternion error: $e_q = log(Q_g * Q^{-1})$, $\alpha_z = ' num2str(a_z) '$'], 'interpreter','latex', 'fontsize',18); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',17); end
   hold off;
end

figure;
Q_labels = {'$\eta$','$\epsilon_1$', '$\epsilon_2$', '$\epsilon_3$'};
Qd_labels = {'$\eta_d$','$\epsilon_{d,1}$', '$\epsilon_{d,2}$', '$\epsilon_{d,3}$'};
for i=1:4
   subplot(4,1,i);
   hold on;
   plot(Time, Q_data(i,:), 'LineWidth', line_width);
   plot(Timed, Qd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({Q_labels{i}, Qd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Unit Quaternion', 'interpreter','latex', 'fontsize',17); end
   if (i==4), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

figure;
vRot_labels = {'$\omega_x$','$\omega_y$', '$\omega_z$'};
vRotd_labels = {'$\omega_{d,x}$','$\omega_{d,y}$', '$\omega_{d,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, vRot_data(i,:), 'LineWidth', line_width);
   plot(Timed, vRotd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({vRot_labels{i}, vRotd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Rotational Velocity', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

figure;
dvRot_labels = {'$\dot{\omega}_x$','$\dot{\omega}_y$', '$\dot{\omega}_z$'};
dvRotd_labels = {'$\dot{\omega}_{d,x}$','$\dot{\omega}_{d,y}$', '$\dot{\omega}_{d,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, dvRot_data(i,:), 'LineWidth', line_width);
   plot(Timed, dvRotd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({dvRot_labels{i}, dvRotd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Rotational Acceleration', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end


Timed = Timed(:,1:10:end);
Qd_data = Qd_data(:,1:10:end);

Time = Time(:,1:10:end);
Q_data = Q_data(:,1:10:end);

ax = axes('Parent',figure());

Pd_data = [Timed; zeros(1,length(Timed)); zeros(1,length(Timed))];

plot_3Dpath_with_orientFrames(Pd_data, Qd_data, 'axes',ax, 'LineWidth',3.0, 'LineStyle','-', 'numberOfFrames',25, 'frameScale',0.2, 'frameLineWidth',2.0, ...
    'FrameXAxisLegend','$x_d$-axis', 'FrameYAxisLegend','$y_d$-axis', 'FrameZAxisLegend','$z_d$-axis', 'LineLegend','demo-path', 'LegendFontSize',14, ...
    'VideoCapture',false, 'VideoFilename','demo', 'animated',true);
    
P_data = [Time; 0.6*ones(1,length(Time)); zeros(1,length(Time))];
% P_data = P_data - repmat([0 3 0]', 1, size(P_data,2));
plot_3Dpath_with_orientFrames(P_data, Q_data, 'axes',ax, 'LineWidth',2.0, 'LineStyle','-', 'LineColor',0.8*[0.93 0.69 0.13], ....
    'numberOfFrames',25, 'frameScale',0.2, 'frameLineWidth',2.0, ...
    'frameXAxisColor', [1 0 1], 'frameYAxisColor', [0 0.5 0], 'frameZAxisColor', [0 1 1], 'frameLineStyle','-', 'animated',true, ...
    'VideoCapture',false, 'VideoFilename','sim', ...
    'FrameXAxisLegend','$x$-axis', 'FrameYAxisLegend','$y$-axis', 'FrameZAxisLegend','$z$-axis', 'LineLegend','sim-path', 'LegendFontSize',14);


end


