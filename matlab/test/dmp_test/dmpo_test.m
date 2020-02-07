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

Ts = Timed(2)-Timed(1);

%% initialize DMP
a_z = 600;
b_z = a_z/4;
train_method = DMP_TRAIN.LS;
can_clock_ptr = CanonicalClock();
shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
% shape_attr_gat_ptr = LinGatingFunction(1.0, 0.02);
N_kernels = [40; 40; 40];
dmp_o = DMPo(DMP_TYPE.STD, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);

disp('DMPo training...')
tic
offline_train_mse = dmp_o.train(train_method, Timed, Qd_data, vRotd_data, dvRotd_data);
offline_train_mse
toc

%% DMP simulation
disp('DMP simulation...');
tic
Qd0 = Qd_data(:,1);
Q0 = Qd0;
Qgd = Qd_data(:,end);
ks = [1.0; 1.0; 1.0];
e0 = ks.*quatLog( quatProd( Qgd, quatInv(Q0) ) );
Qg = quatProd(quatExp(e0), Q0);
Qg = quatProd(rotm2quat(rotx(0))', Qgd);
T = 1.0*Timed(end);
dt = Ts;

ks = quatLog( quatProd( Qg, quatInv(Q0) ) ) ./ quatLog( quatProd( Qgd, quatInv(Q0) ) );

ks

[Time, Q_data, vRot_data, dvRot_data] = simulateDMPo_in_quat_space(dmp_o, Q0, Qg, T, dt);
% [Time, Q_data, vRot_data, dvRot_data] = simulateDMPo_in_log_space(dmp_o, Q0, Qg, T, dt);
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
legend('$dmp_o$', '$k_s*demo$');
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


