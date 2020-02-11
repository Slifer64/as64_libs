addpath('utils/');

filename = 'data/dmpo_sim_data.bin';
%% Load sim data
in = fopen(filename,'r');
if (in < 0), error(['Failed to load ''' filename '''']); end

Timed = read_mat(in, true);
Qd_data = read_mat(in, true);
vRotd_data = read_mat(in, true);
dvRotd_data = read_mat(in, true);
Time = read_mat(in, true);
Q_data = read_mat(in, true);
vRot_data = read_mat(in, true);
dvRot_data = read_mat(in, true);

fclose(in);

%% Plot results


Qgd = Qd_data(:,end);
Q0d = Qd_data(:,1);

Qg = Q_data(:,end);
Q0 = Q_data(:,1);

ks = quatLog( quatProd(Qg, quatInv(Q0)) ) ./ quatLog( quatProd(Qgd, quatInv(Q0d)) );

zeros(3, size(Qd_data,2));
qd_data = zeros(3, size(Qd_data,2));
for j=1:size(qd_data,2)
    qd_data(:,j) = quatLog( quatProd( Qd_data(:,j), quatInv(Q0d) ) );
end

Qg = Q_data(:,end);
q_data = zeros(3, size(Q_data,2));
for j=1:size(q_data,2)
    q_data(:,j) = quatLog( quatProd( Q_data(:,j), quatInv(Q0) ) );
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
   if (i==1), legend({'sim', 'ks*demo'}, 'interpreter','latex', 'fontsize',16, 'Position',[0.7 0.78 0.27 0.15]); end
   if (i==1), title('Quaternion error: $e_q = log(Q_g * Q^{-1})$', 'interpreter','latex', 'fontsize',18); end
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