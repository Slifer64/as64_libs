
path = strrep(mfilename('fullpath'), 'plot_DMPpEKFc_results','');
addpath([path 'utils/']);

addpath([path '/utils/']);

filename = [path '/data/sim/sim_DMPpEKFc_results.bin'];
%% Load sim data
in = fopen(filename,'r');
if (in < 0), error(['Failed to load ''' filename '''']); end
  
Time = read_mat(in, true);
Yg = read_mat(in, true);
Yg_data = read_mat(in, true);
tau = read_scalar(in, true, 'double');
tau_data = read_mat(in, true);
Sigma_theta_data = read_mat(in, true);
F_data = read_mat(in, true);
Y_data = read_mat(in, true);
dY_data = read_mat(in, true);

plot_1sigma = false;

fclose(in);

%% Plot results

figure
hold on;
plot(Time, Sigma_theta_data(1,:), 'LineWidth',2.0, 'Color','red');
plot(Time, Sigma_theta_data(2,:), 'LineWidth',2.0, 'Color','green');
plot(Time, Sigma_theta_data(3,:), 'LineWidth',2.0, 'Color','blue');
plot(Time, Sigma_theta_data(4,:), 'LineWidth',2.0, 'Color',[0.93 0.69 0.13]);
legend({'$x$','$y$','$z$','$\tau$'}, 'interpreter','latex', 'fontsize',15);
title('$\sigma_{\theta}$', 'interpreter','latex', 'fontsize',15);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
hold off;


%% ==============================================================
%% ==========   Target and time scaling est plot ================

fontsize = 16;
linewidth = 1.5;

D = length(Yg);

axis_name = {'x', 'y', 'z'};

goal_color = {[1 0 0], [0 0.5 0], [0 0 1]};
goal_est_color = {[1 0 1], [0 1 0], [0 1 1]};
pos_color = {[0.85 0.7 1], [0.75 0.75 0], [0 0.45 0.75]};
tau_color = [0 0 1];
tau_hat_color = [0.85 0.33 0.1];

est_fig = figure('Position', [150 150 800 1000]);
est_fig_ax = cell(3,1);
est_fig_ax{1} = axes('Parent',est_fig, 'Position',[0.13 0.595 0.775 0.33]);
hold(est_fig_ax{1},'on');
legend_labels = {};
for i=1:D
    plot([Time(1) Time(end)],[Yg(i) Yg(i)], 'LineStyle','--', 'Color',goal_color{i} ,'LineWidth',2, 'Parent',est_fig_ax{1});
    plot(Time,Yg_data(i,:), 'LineStyle','-', 'Color',goal_est_color{i}, 'LineWidth',linewidth, 'Parent',est_fig_ax{1});
    plot(Time,Y_data(i,:), 'LineStyle','-.', 'Color',pos_color{i}, 'LineWidth',linewidth, 'Parent',est_fig_ax{1});
    legend_labels = [legend_labels, ['$\mathbf{y}_{g,' axis_name{i} '}$'], ['$\hat{\mathbf{y}}_{g,' axis_name{i} '}$'], ['$\mathbf{y}_{' axis_name{i} '}$']];
    if (plot_1sigma)
        plot(Time,Yg_data(i,:)+Sigma_theta_data(i,:), 'LineStyle','-.', 'Color','cyan', 'LineWidth',linewidth, 'Parent',est_fig_ax{1});
        plot(Time,Yg_data(i,:)-Sigma_theta_data(i,:), 'LineStyle','-.', 'Color','cyan',  'LineWidth',linewidth, 'Parent',est_fig_ax{1});
        legend_labels = [legend_labels, ['$\pm1\sigma$']];
    end
    ylabel('[$m$]','interpreter','latex','fontsize',fontsize, 'Parent',est_fig_ax{1});
    axis tight;
end
legend(est_fig_ax{1}, legend_labels,'interpreter','latex','fontsize',fontsize, 'Position',[0.005 0.95 0.99 0.036], 'Orientation','horizontal');
hold(est_fig_ax{1},'off');

est_fig_ax{2} = axes('Parent',est_fig, 'Position',[0.13 0.347 0.775 0.189]);
hold(est_fig_ax{2},'on');
plot([Time(1) Time(end)],[tau tau], 'LineStyle','--', 'Color',tau_color, 'LineWidth',2, 'Parent',est_fig_ax{2});
plot(Time,tau_data, 'LineStyle','-', 'Color',tau_hat_color, 'LineWidth',linewidth, 'Parent',est_fig_ax{2});
legend_labels = {['$\tau$'], ['$\hat{\tau}$']};
%legend_labels = {['$\tau$']};
if (plot_1sigma)
    plot(Time,tau_data+Sigma_theta_data(end,:),'c-.', 'LineWidth',linewidth, 'Parent',est_fig_ax{2});
    plot(Time,tau_data-Sigma_theta_data(end,:),'c-.', 'LineWidth',linewidth, 'Parent',est_fig_ax{2});
    legend_labels = [legend_labels, ['$\pm1\sigma$']];
end
legend(est_fig_ax{2}, legend_labels,'interpreter','latex','fontsize',fontsize);
ylabel('$\tau$ [$s$]','interpreter','latex','fontsize',fontsize, 'Parent',est_fig_ax{2});
axis tight;
hold(est_fig_ax{2},'off');

f_norm = zeros(size(F_data,2),1);
for i=1:length(f_norm)
    f_norm(i) = norm(F_data(:,i));
end

est_fig_ax{3} = axes('Parent',est_fig, 'Position',[0.13 0.0845 0.775 0.216]);
hold(est_fig_ax{3},'on');
plot(Time, f_norm,'b-', 'LineWidth',linewidth, 'Parent',est_fig_ax{3});
ylabel(['$||\mathbf{f}_{ext}||$ [$N$]'],'interpreter','latex','fontsize',fontsize, 'Parent',est_fig_ax{3});
xlabel('time [$s$]','interpreter','latex','fontsize',fontsize, 'Parent',est_fig_ax{3});
axis tight;
hold(est_fig_ax{3},'off');

    
%% ==================================================
%% ===============   Calc effort  ===================
    
dt = Time(2)-Time(1);

effort = 0.0;
sum_f2 = 0.0;
P = zeros(length(Time),1);
F_square = zeros(length(Time),1);
for i=1:size(F_data,2)
    P(i) = abs(F_data(:,i)'*dY_data(:,i));
    effort = effort + abs(P(i))*dt;
    F_square(i) = F_data(:,i)'*F_data(:,i);
    sum_f2 = sum_f2 + F_square(i)*dt;
end
goal_err = norm(Yg-Yg_data(:,end));
    
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


