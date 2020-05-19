function u_o()

set_matlab_utils_path();

%% Load training data
fid = FileIO('train_data.bin', FileIO.in);
Timed = fid.read('Timed');
Qd_data = fid.read('Qd_data');
vRotd_data = fid.read('vRotd_data');
dvRotd_data = fid.read('dvRotd_data');

Ts = Timed(2)-Timed(1);

simulateGMPo = @simulateGMPo_in_log_space; % simulateGMPo_in_log/quat_space

%% initialize and train GMP
train_method = 'LS';
N_kernels = 30;
kernels_std_scaling = 1;
gmp_o = GMPo(N_kernels, 0.1, 20, kernels_std_scaling);
tic
offline_train_mse = gmp_o.train(train_method, Timed, Qd_data);
offline_train_mse
toc

% gmp_o.exportToFile('data/gmp_o_model.bin');
% gmp_o = GMPo.importFromFile('data/gmp_o_model.bin');

%% DMP simulation
disp('GMP simulation...');
tic
Qd0 = Qd_data(:,1);
Q0 = Qd0;
Qgd = Qd_data(:,end);
ks = 1;
kt = 1;
e0 = ks*quatLog( quatProd( Qgd, quatInv(Qd0) ) );
Qg = quatProd(quatExp(e0), Q0); %quatExp(1.0*quatLog(Qd_data(:,end)));
T = kt*Timed(end);
dt = Ts;
[Time, Q_data, vRot_data, dvRot_data] = simulateGMPo(gmp_o, Q0, Qg, T, dt);
toc


%% Plot results

Pqd_data = zeros(3, size(Qd_data,2));
for j=1:size(Pqd_data,2)
    Pqd_data(:,j) = GMPo.quat2q(Qd_data(:,j), Qd0);
end

Pq_data = zeros(3, size(Q_data,2));
for j=1:size(Pq_data,2)
    Pq_data(:,j) = GMPo.quat2q(Q_data(:,j), Q0);
end


Pq_data2 = zeros(3, size(Q_data,2));
for j=1:size(Pq_data2,2)
    Pq_data2(:,j) = gmp_o.getYd(Time(j)/T);
end


line_width = 2.5;
 
figure('Position', [200 200 600 500]);
y_labels = {'$e_{q,x}$','$e_{q,y}$', '$e_{q,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, Pq_data(i,:), 'LineWidth', line_width);
   plot(Time, Pq_data2(i,:), 'LineWidth', line_width);
   plot(Timed, ks*Pqd_data(i,:), 'LineWidth', line_width, 'LineStyle','--');
   ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',20);
   axis tight;
   if (i==1), legend({'gmp', 'wsog', 'demo'}, 'interpreter','latex', 'fontsize',16, 'Position',[0.7 0.78 0.27 0.15]); end
   if (i==1), title('Quaternion error: $e_q = log(Q * Q_0^{-1})$', 'interpreter','latex', 'fontsize',18); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',17); end
   hold off;
end

return


figure;
hold on;
plot3(Pq_data(1,:), Pq_data(2,:), Pq_data(3,:), 'LineWidth', line_width, 'LineStyle','-');
plot3(ks*Pqd_data(1,:), ks*Pqd_data(2,:), ks*Pqd_data(3,:), 'LineWidth', line_width, 'LineStyle','--');
plot3(Pqd_data(1,:), Pqd_data(2,:), Pqd_data(3,:), 'LineWidth', line_width, 'LineStyle','--');
legend('gmp', 'k_s * demo', 'demo');
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


