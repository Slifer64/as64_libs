clc;
close all;
clear;

set_matlab_utils_path();

%% Load training data

path = strrep(mfilename('fullpath'), 'dmp_eo_scale_test','');

load([path 'data/train_data3.mat'], 'Data');
% load('data/orient_data.mat', 'Data');

Timed = Data.Time;
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
shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
% shape_attr_gat_ptr = LinGatingFunction(1.0, 0.02);
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
Q0d = Qd_data(:,1);
Qgd = Qd_data(:,end);
% Q0 = Q0d;
Q0 = quatProd(rotm2quat(rotx(80)*rotz(40))', Q0d);
ks = [2.0; 1.0; 1.5];
e0 = ks.*quatLog( quatProd( Qgd, quatInv(Q0d) ) );
Qg = quatProd(quatExp(e0), Q0); %quatExp(1.0*quatLog(Qd_data(:,end)));
% Qg = quatProd(rotm2quat(rotx(0))', Qgd);
T = 1.0*Timed(end);
dt = Ts;

ks = quatLog( quatProd( Qg, quatInv(Q0) ) ) ./ quatLog( quatProd( Qgd, quatInv(Q0d) ) );

ks

[Time, Q_data, vRot_data, dvRot_data] = simulateDMPeo_in_quat_space(dmp_o, Q0, Qg, T, dt);
toc

disp('DMP old simulation...');
tic
[Time2, Q_data2, vRot_data2, dvRot_data2] = simulateDMPeo_in_eo_space(dmp_o, Q0, Qg, T, dt);
toc

% Data = struct('Time',Time, 'Quat',Q_data, 'RotVel',vRot_data, 'RotAccel',dvRot_data);
% save([path 'data/orient_data.mat'], 'Data');

%% Plot results

Qgd = Qd_data(:,end);
Pqd_data = zeros(3, size(Qd_data,2));
for j=1:size(Pqd_data,2)
    Pqd_data(:,j) = quatLog( quatProd(Qgd, quatInv(Qd_data(:,j))) );
end

Pq_data = zeros(3, size(Q_data,2));
for j=1:size(Pq_data,2)
    Pq_data(:,j) = quatLog( quatProd(Qg, quatInv(Q_data(:,j))) );
end

Pq_data2 = zeros(3, size(Q_data2,2));
for j=1:size(Pq_data2,2)
    Pq_data2(:,j) = quatLog( quatProd(Qg, quatInv(Q_data2(:,j))) );
end

line_width = 2.5;
 
figure('Position', [200 200 600 500]);
y_labels = {'$e_{q,x}$','$e_{q,y}$', '$e_{q,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, Pq_data(i,:), 'LineWidth', line_width);
   plot(Time2, Pq_data2(i,:), 'LineWidth', line_width, 'LineStyle',':');
   plot(Timed, ks(i)*Pqd_data(i,:), 'LineWidth', line_width, 'LineStyle','--');
   ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',20);
   axis tight;
   if (i==1), legend({'new', 'old', 'ks*demo'}, 'interpreter','latex', 'fontsize',16, 'Position',[0.7 0.78 0.27 0.15]); end
   if (i==1), title(['Quaternion error: $e_q = log(Q_g * Q^{-1})$, $\alpha_z = ' num2str(a_z) '$'], 'interpreter','latex', 'fontsize',18); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',17); end
   hold off;
end

% figure;
% hold on;
% plot3(Pq_data(1,:), Pq_data(2,:), Pq_data(3,:), 'LineWidth', line_width, 'LineStyle','-');
% plot3(ks(1)*Pqd_data(1,:), ks(2)*Pqd_data(2,:), ks(3)*Pqd_data(3,:), 'LineWidth', line_width, 'LineStyle','--');
% legend('proposed', 'demo');
% hold off;

figure;
hold on;
plot3(Pq_data(1,:), Pq_data(2,:), Pq_data(3,:), 'LineWidth', line_width, 'LineStyle','-');
plot3(Pq_data2(1,:), Pq_data2(2,:), Pq_data2(3,:), 'LineWidth', line_width, 'LineStyle','-');
plot3(Pqd_data(1,:), Pqd_data(2,:), Pqd_data(3,:), 'LineWidth', line_width, 'LineStyle','--');
legend('proposed','old','ks*demo');
hold off;

figure;
Q_labels = {'$\eta$','$\epsilon_1$', '$\epsilon_2$', '$\epsilon_3$'};
Q2_labels = {'old $\eta$','old $\epsilon_1$', 'old $\epsilon_2$', 'old $\epsilon_3$'};
Qd_labels = {'$\eta_d$','$\epsilon_{d,1}$', '$\epsilon_{d,2}$', '$\epsilon_{d,3}$'};

Qd_data2 = Qd_data;
for j=1:size(Qd_data,2)
   Qd_data2(:,j) = quatProd( quatExp(ks.*quatLog( quatProd(Qd_data(:,j),quatInv(Q0d))) ), Q0);
end

Qd_data3 = Qd_data;
for j=1:size(Qd_data,2)
   Qd_data3(:,j) = quatProd( quatInv(quatExp(ks.*quatLog( quatProd(Qgd, quatInv(Qd_data(:,j)))) )), Qg);
end

for i=1:4
   subplot(4,1,i);
   hold on;
   plot(Time, Q_data(i,:), 'LineWidth', line_width);
   plot(Time2, Q_data2(i,:), 'LineWidth', line_width, 'LineStyle','--');
   plot(Timed, Qd_data2(i,:), 'LineWidth', line_width, 'LineStyle',':');
   plot(Timed, Qd_data3(i,:), 'LineWidth', line_width, 'LineStyle','-.');
   legend({Q_labels{i}, Q2_labels{i}, Qd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Unit Quaternion', 'interpreter','latex', 'fontsize',17); end
   if (i==4), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

% figure;
% vRot_labels = {'$\omega_x$','$\omega_y$', '$\omega_z$'};
% vRot2_labels = {'old $\omega_x$','old $\omega_y$', 'old $\omega_z$'};
% vRotd_labels = {'$\omega_{d,x}$','$\omega_{d,y}$', '$\omega_{d,z}$'};
% for i=1:3
%    subplot(3,1,i);
%    hold on;
%    plot(Time, vRot_data(i,:), 'LineWidth', line_width);
%    plot(Time2, vRot_data2(i,:), 'LineWidth', line_width, 'LineStyle','--');
%    plot(Timed, vRotd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
%    legend({vRot_labels{i}, vRot2_labels{i}, vRotd_labels{i}}, 'interpreter','latex', 'fontsize',15);
%    if (i==1), title('Rotational Velocity', 'interpreter','latex', 'fontsize',17); end
%    if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
%    hold off;
% end
% 
% figure;
% dvRot_labels = {'$\dot{\omega}_x$','$\dot{\omega}_y$', '$\dot{\omega}_z$'};
% dvRot2_labels = {'old $\dot{\omega}_x$','old $\dot{\omega}_y$', 'old $\dot{\omega}_z$'};
% dvRotd_labels = {'$\dot{\omega}_{d,x}$','$\dot{\omega}_{d,y}$', '$\dot{\omega}_{d,z}$'};
% for i=1:3
%    subplot(3,1,i);
%    hold on;
%    plot(Time, dvRot_data(i,:), 'LineWidth', line_width);
%    plot(Time2, dvRot_data2(i,:), 'LineWidth', line_width, 'LineStyle','--');
%    plot(Timed, dvRotd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
%    legend({dvRot_labels{i}, dvRot2_labels{i}, dvRotd_labels{i}}, 'interpreter','latex', 'fontsize',15);
%    if (i==1), title('Rotational Acceleration', 'interpreter','latex', 'fontsize',17); end
%    if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
%    hold off;
% end



