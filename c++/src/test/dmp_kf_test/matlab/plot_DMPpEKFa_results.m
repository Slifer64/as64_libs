
path = strrep(mfilename('fullpath'), 'plot_DMPpEKFa_results','');
addpath([path 'utils/']);

addpath([path '/utils/']);

filename = [path '/data/sim/sim_DMPpEKFa_results.bin'];
%% Load sim data
in = fopen(filename,'r');
if (in < 0), error(['Failed to load ''' filename '''']); end
  
Time = read_mat(in, true);
pg = read_mat(in, true);
pg_data = read_mat(in, true);
tau = read_scalar(in, true, 'double');
tau_data = read_mat(in, true);
Sigma_theta_data = read_mat(in, true);
F_data = read_mat(in, true);
p_data = read_mat(in, true);
p_dot_data = read_mat(in, true);
p_hat_data = read_mat(in, true);
p_dot_hat_data = read_mat(in, true);

plot_1sigma = false;

fclose(in);

f_norm = zeros(size(F_data,2),1);
for i=1:length(f_norm)
    f_norm(i) = norm(F_data(:,i));
end

%% *******  Plot results  *********

%% =============================================
%% ==========   Unit Quaternion ================

figure
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, p_data(i,:), 'LineWidth',2, 'Color','blue');
   plot(Time, p_hat_data(i,:), 'LineWidth',2, 'Color','magenta');
   if (i==1), legend('actual','estimate'); end
   if (i==1), title('Position'); end
   hold off;
end


%% =================================================
%% ==========   Rotational Velocity ================

figure
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, p_dot_data(i,:), 'LineWidth',2, 'Color','blue');
   plot(Time, p_dot_hat_data(i,:), 'LineWidth',2, 'Color','magenta');
   if (i==1), legend('actual','estimate'); end
   if (i==1), title('Velocity'); end
   hold off;
end

%% =========================================
%% ==========   Sigma theta ================

figure
hold on;
for i=1:10
    plot(Time, Sigma_theta_data(i,:), 'LineWidth',2.0);
end
legend({'$\dot{x}$','$\dot{y}$','$\dot{z}$','$x$','$y$','$z$','$g_x$','$g_y$','$g_z$','$\tau$'}, 'interpreter','latex', 'fontsize',15);
title('$\sigma_{\theta}$', 'interpreter','latex', 'fontsize',15);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
hold off;

    
%% ==============================================================
%% ==========   Target and time scaling est plot ================

fontsize = 16;
linewidth = 1.5;

D = length(pg);

axis_name = {'x', 'y', 'z'};

goal_color = {[1 0 0], [0 0.5 0], [0 0 1], [0.85 0.33 0.1]};
goal_est_color = {[1 0 1], [0 1 0], [0 1 1], [0.93 0.69 0.13]};
pos_color = {[0.85 0.7 1], [0.75 0.75 0], [0 0.45 0.75], [1.0 0.84 0.0]};
tau_color = [0 0 1];
tau_hat_color = [0.85 0.33 0.1];

n = size(pg_data,2);

figure;
for i=1:3
    subplot(5,1,i);
    hold on;
    plot([Time(1) Time(end)],[pg(i) pg(i)], 'LineStyle','--', 'Color','red' ,'LineWidth',2);
    plot(Time,pg_data(i,:), 'LineStyle','-', 'Color','blue', 'LineWidth',linewidth);
    plot(Time,p_data(i,:), 'LineStyle','-.', 'Color',[0 0.7 0], 'LineWidth',linewidth);
    legend_labels = {['$\mathbf{p}_{g,' axis_name{i} '}$'], ['$\hat{\mathbf{p}}_{g,' axis_name{i} '}$'], ['$\mathbf{p}_{' axis_name{i} '}$']};
    if (plot_1sigma)
        plot(Time,pg_data(i,:)+Sigma_theta_data(i,:),'c-.', 'LineWidth',linewidth);
        plot(Time,pg_data(i,:)-Sigma_theta_data(i,:),'c-.', 'LineWidth',linewidth);
        legend_labels = [legend_labels, ['$\pm1\sigma$']];
    end
    % ylabel('[$m$]','interpreter','latex','fontsize',fontsize);
    legend(legend_labels,'interpreter','latex','fontsize',fontsize);
    axis tight;
    hold off;
end

subplot(5,1,4);
hold on;
plot([Time(1) Time(end)],[tau tau], 'LineStyle','--', 'Color',tau_color, 'LineWidth',2);
plot(Time,tau_data, 'LineStyle','-', 'Color',tau_hat_color, 'LineWidth',linewidth);
legend_labels = {['$\tau$'], ['$\hat{\tau}$']};
if (plot_1sigma)
    plot(Time,tau_data+Sigma_theta_data(end,:),'c-.', 'LineWidth',linewidth);
    plot(Time,tau_data-Sigma_theta_data(end,:),'c-.', 'LineWidth',linewidth);
    legend_labels = [legend_labels, ['$\pm1\sigma$']];
end
legend(legend_labels,'interpreter','latex','fontsize',fontsize);
ylabel('$\tau$ [$s$]','interpreter','latex','fontsize',fontsize);
axis tight;
hold off;

subplot(5,1,5);
hold on;
plot(Time, f_norm,'b-', 'LineWidth',linewidth);
ylabel('$||\mathbf{f}_{ext}||$ [$N$]','interpreter','latex','fontsize',fontsize);
xlabel('time [$s$]','interpreter','latex','fontsize',fontsize);
axis tight;
hold off;

    
%% ==================================================
%% ===============   Calc effort  ===================
    
dt = Time(2)-Time(1);

effort = 0.0;
sum_f2 = 0.0;
P = zeros(length(Time),1);
F_square = zeros(length(Time),1);
for i=1:size(F_data,2)
    P(i) = abs(F_data(:,i)'*p_dot_data(:,i));
    effort = effort + abs(P(i))*dt;
    F_square(i) = F_data(:,i)'*F_data(:,i);
    sum_f2 = sum_f2 + F_square(i)*dt;
end
goal_err = norm(pg-pg_data(:,end));
    
%% ================================================
%% ================  Plot effort  =================

linewidth = 2.0;
effort_fig = figure;
effort_fig_ax = cell(2,1);
effort_fig_ax{1} = subplot(2,1,1);
plot(Time, P, 'LineWidth',linewidth)
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',fontsize);
ylabel('Power [$Watt$]', 'interpreter','latex', 'fontsize',fontsize);
axis tight;

effort_fig_ax{2} = subplot(2,1,2);
plot(Time, F_square, 'LineWidth',linewidth)
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',fontsize);
ylabel('$||\mathbf{f}_{ext}||^2$ [$N$]', 'interpreter','latex', 'fontsize',fontsize);
axis tight;
    
    
%% ================================================
%% ================  Display results  =============
    
metrics_results = {'Work', 'Power_F', 'target error'; effort, sum_f2, goal_err};

disp(metrics_results);
