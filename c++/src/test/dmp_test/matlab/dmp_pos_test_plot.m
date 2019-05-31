
addpath('utils/');

filename = 'data/dmp_pos_sim_data.bin';
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

fclose(in);

%% Plot results

Pgd = Pd_data(:,end);
Pqd_data = zeros(3, size(Pd_data,2));
for j=1:size(Pqd_data,2)
    Pqd_data(:,j) = Pgd - Pd_data(:,j);
end

Pg = P_data(:,end);
Pq_data = zeros(3, size(P_data,2));
for j=1:size(Pq_data,2)
    Pq_data(:,j) = Pg - P_data(:,j);
end

line_width = 2.5;
 
figure('Position', [200 200 600 500]);
y_labels = {'$e_{q,x}$','$e_{q,y}$', '$e_{q,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, Pq_data(i,:), 'LineWidth', line_width);
   plot(Timed, Pqd_data(i,:), 'LineWidth', line_width, 'LineStyle','--');
   ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',20);
   axis tight;
   if (i==1), legend({'pos DMP', 'demo'}, 'interpreter','latex', 'fontsize',16, 'Position',[0.7 0.78 0.27 0.15]); end
   if (i==1), title('Position error: $e_p = P_g - P$', 'interpreter','latex', 'fontsize',18); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',17); end
   hold off;
end

figure;
hold on;
plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth', line_width, 'LineStyle','-');
plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth', line_width, 'LineStyle','--');
hold off;

figure;
P_labels = {'$x$','$y$', '$z$'};
Pd_labels = {'$x_d$','$y_d$', '$z_d$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, P_data(i,:), 'LineWidth', line_width);
   plot(Timed, Pd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({P_labels{i}, Pd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Cartesian Position', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

figure;
dP_labels = {'$p_x$','$p_y$', '$p_z$'};
dPd_labels = {'$p_{d,x}$','$p_{d,y}$', '$p_{d,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, dP_data(i,:), 'LineWidth', line_width);
   plot(Timed, dPd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({dP_labels{i}, dPd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Cartesian Velocity', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

figure;
ddP_labels = {'$\dot{p}_x$','$\dot{p}_y$', '$\dot{p}_z$'};
ddPd_labels = {'$\dot{p}_{d,x}$','$\dot{p}_{d,y}$', '$\dot{p}_{d,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, ddP_data(i,:), 'LineWidth', line_width);
   plot(Timed, ddPd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({ddP_labels{i}, ddPd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Cartesian Acceleration', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end


