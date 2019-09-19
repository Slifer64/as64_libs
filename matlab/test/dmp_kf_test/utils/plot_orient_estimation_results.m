function plot_orient_estimation_results(Time, Qg, Qg_data, tau, tau_data, Sigma_theta_data, F_data, plot_1sigma, Q_data, vRot_data)

f_norm = zeros(size(F_data,2),1);
for i=1:length(f_norm)
    f_norm(i) = norm(F_data(:,i));
end
    
%% ==============================================================
%% ==========   Target and time scaling est plot ================

fontsize = 16;
linewidth = 1.5;

D = length(Qg);

axis_name = {'x', 'y', 'z'};

tau_color = [0 0 1];
tau_hat_color = [0.85 0.33 0.1];

n = size(Qg_data,2);
eQg_data = zeros(3,n);
eQ_data = zeros(3,n);

for j=1:n
    eQg_data(:,j) = quatLog(quatDiff(Qg,Qg_data(:,j)));
    eQ_data(:,j) = quatLog(quatDiff(Qg,Q_data(:,j)));
end

figure;
for i=1:3
    subplot(5,1,i);
    hold on;
    plot([Time(1) Time(end)],[0 0], 'LineStyle','--', 'Color','red' ,'LineWidth',2);
    plot(Time,eQg_data(i,:), 'LineStyle','-', 'Color','blue', 'LineWidth',linewidth);
    plot(Time,eQ_data(i,:), 'LineStyle','-.', 'Color',[0 0.7 0], 'LineWidth',linewidth);
    legend_labels = {['$\mathbf{eQ}_{g,' axis_name{i} '}$'], ['$\hat{\mathbf{eQ}}_{g,' axis_name{i} '}$'], ['$\mathbf{Q}_{' axis_name{i} '}$']};
    if (plot_1sigma)
        plot(Time,eQg_data(i,:)+Sigma_theta_data(i,:),'c-.', 'LineWidth',linewidth);
        plot(Time,eQg_data(i,:)-Sigma_theta_data(i,:),'c-.', 'LineWidth',linewidth);
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
        P(i) = abs(F_data(:,i)'*vRot_data(:,i));
        effort = effort + abs(P(i))*dt;
        F_square(i) = F_data(:,i)'*F_data(:,i);
        sum_f2 = sum_f2 + F_square(i)*dt;
    end
    goal_err = norm(Qg-Qg_data(:,end));
    
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
    
    
end
