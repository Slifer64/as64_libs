function dmpo_test()

set_matlab_utils_path();

%% Load training data

path = strrep(mfilename('fullpath'), 'dmpo_test','');

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

% Read train data from binary file
fid = fopen('data/dmpo_train_data.bin','r');
Timed = read_mat(fid, true);
Qd_data = read_mat(fid, true);
vRotd_data = read_mat(fid, true);
dvRotd_data = read_mat(fid, true);
fclose(fid);

Ts = Timed(2)-Timed(1);

%% initialize DMP
a_z = 5;
b_z = a_z/4;
train_method = DMP_TRAIN.LS;
can_clock_ptr = CanonicalClock();
shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
% shape_attr_gat_ptr = LinGatingFunction(1.0, 0.02);
N_kernels = [40; 40; 40];
simulateDMPo = @simulateDMPo_in_log_space; % simulateDMPo_in_log/quat_space
dmp_o = DMPo(DMP_TYPE.STD, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);

disp('DMPo training...')
tic
offline_train_mse = dmp_o.train(train_method, Timed, Qd_data, vRotd_data, dvRotd_data);
offline_train_mse
toc

% % read c++ DMP weights
% fid = fopen('/home/slifer/slifer/as64_libs/c++/devel/lib/dmp_test/weights.bin','r');
% w = read_mat(fid, true);
% q_data = read_mat(fid, true);
% qdot_data = read_mat(fid, true);
% qddot_data = read_mat(fid, true);
% Quat_data = read_mat(fid, true);
% fclose(fid);
% 
% % read matlab DMP weights
% fid = fopen('weights.bin','r');
% w2 = read_mat(fid, true);
% q_data2 = read_mat(fid, true);
% qdot_data2 = read_mat(fid, true);
% qddot_data2 = read_mat(fid, true);
% Quat_data2 = read_mat(fid, true);
% fclose(fid);
% 
% w_err = w - w2;
% q_err = q_data - q_data2;
% dq_err = qdot_data - qdot_data2;
% ddq_err = qddot_data - qddot_data2;
% Quat_err = Quat_data - Quat_data2;
% 
% figure;
% bar(w_err);
% 
% figure;
% for i=1:3
%     subplot(3,1,i);
%     plot(q_err(i,:), 'LineWidth',2, 'Color','red'); 
% end
% 
% figure;
% for i=1:3
%     subplot(3,1,i);
%     plot(dq_err(i,:), 'LineWidth',2, 'Color','red'); 
% end
% 
% figure;
% for i=1:3
%     subplot(3,1,i);
%     plot(ddq_err(i,:), 'LineWidth',2, 'Color','red'); 
% end
% 
% figure;
% for i=1:4
%     subplot(4,1,i);
%     plot(Quat_err(i,:), 'LineWidth',2, 'Color','red'); 
% end
% 
% 
% fid = fopen('weights2.bin','w');
% write_mat(w2, fid, true);
% fclose(fid);

% return

%% DMP simulation
disp('DMP simulation...');
tic
Qd0 = Qd_data(:,1);
Qgd = Qd_data(:,end);
Q0 = quatProd(rotm2quat(rotx(80)*rotz(40))', Qd0);
ks = [2.0; 1.5; 1.8];
e0 = ks.*quatLog( quatProd( Qgd, quatInv(Qd0) ) );
Qg = quatProd(quatExp(e0), Q0);
T = 1.0*Timed(end);
dt = Ts;

ks = quatLog( quatProd( Qg, quatInv(Q0) ) ) ./ quatLog( quatProd( Qgd, quatInv(Qd0) ) );

ks

[Time, Q_data, vRot_data, dvRot_data] = simulateDMPo(dmp_o, Q0, Qg, T, dt);
toc


%% Plot results

qd_data = zeros(3, size(Qd_data,2));
for j=1:size(qd_data,2)
    qd_data(:,j) = quatLog( quatProd(Qd_data(:,j), quatInv(Qd0)) );
end

q_data = zeros(3, size(Q_data,2));
for j=1:size(q_data,2)
    q_data(:,j) = quatLog( quatProd(Q_data(:,j), quatInv(Q0)) );
end

line_width = 2.5;
 
figure('Position', [200 200 600 500]);
y_labels = {'$e_{q,x}$','$e_{q,y}$', '$e_{q,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, q_data(i,:), 'LineWidth', line_width);
   plot(Timed, ks(i)*qd_data(i,:), 'LineWidth', line_width, 'LineStyle','--');
   ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',20);
   axis tight;
   if (i==1), legend({'$dmp_o$', '$k_s*demo$'}, 'interpreter','latex', 'fontsize',16, 'Position',[0.7 0.78 0.27 0.15]); end
   if (i==1), title(['Quaternion log: $q = log(Q * Q_0^{-1})$, $\alpha_z = ' num2str(a_z) '$'], 'interpreter','latex', 'fontsize',18); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',17); end
   hold off;
end

figure;
hold on;
plot3(q_data(1,:), q_data(2,:), q_data(3,:), 'LineWidth', line_width, 'LineStyle','-');
plot3(ks(1)*qd_data(1,:), ks(2)*qd_data(2,:), ks(3)*qd_data(3,:), 'LineWidth', line_width, 'LineStyle','--');
plot3(qd_data(1,:), qd_data(2,:), qd_data(3,:), 'LineWidth', line_width, 'LineStyle','-.');
legend({'$dmp_o$', '$k_s*demo$', '$demo$'}, 'interpreter','latex', 'fontsize',15);
hold off;

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


end


