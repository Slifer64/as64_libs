function plot_DMP_EKFc_results(Time, yg_data, tau_data, Sigma_theta_data, y_data, vel_data, yg, F_data, type)

if (strcmpi(type,'pos'))
    legend_labels = {'$x$ [$m$]', '$y$ [$m$]', '$z$ [$m$]'};
    title = 'position';
elseif (strcmpi(type,'orient'))
    legend_labels = {'$q''_x$', '$q''_y$', '$q''_z$'};
    title = 'orientation';
else
    error('Unrecognized type ''%s''', type);
end
    

%% =============== Trim data =====================

%% Plot results
if (~isempty(Sigma_theta_data))
    plotSigma(Time, Sigma_theta_data, title);
end

%% =========================================================================
%% ==========   Orientation Target and time scaling est plot ===============
tau = Time(end);
plotEstResults(Time, y_data, yg_data, tau_data, Sigma_theta_data, yg, tau, legend_labels, title);
    
%% ================================================
%% ================  Plot effort  =================
[effort, mean_f] = plotEffort(Time, F_data, vel_data);

%% ================================================
%% ================  Display results  =============
qg_hat = yg_data(:,end);
target_err = norm(qg_hat - yg);

metrics_results = {'Work', 'mean_force', 'target_error'; effort, mean_f, target_err};

disp(metrics_results);

end


%% =======================================================
%% =======================================================


function plotEstResults(Time, P_data, Pg_data, tau_data, Sigma_theta_data, Pg, tau, y_ax_labels, type, plot_1sigma)

    if (nargin < 10), plot_1sigma = false; end
        
    fontsize = 16;
    linewidth = 1.5;

    figure;
    for i=1:3
        ax = subplot(4,1,i);
        hold(ax,'on');
        plot([Time(1) Time(end)],[Pg(i) Pg(i)], 'LineStyle','--', 'Color','red' ,'LineWidth',2, 'Parent',ax);
        plot(Time,Pg_data(i,:), 'LineStyle','-', 'Color','blue', 'LineWidth',linewidth, 'Parent',ax);
        plot(Time,P_data(i,:), 'LineStyle','-.', 'Color',[0 0.6 0], 'LineWidth',linewidth, 'Parent',ax);
        if (plot_1sigma)
            plot(Time,Pg_data(i,:)+Sigma_theta_data(i,:), 'LineStyle','-.', 'Color','cyan', 'LineWidth',linewidth, 'Parent',ax);
            plot(Time,Pg_data(i,:)-Sigma_theta_data(i,:), 'LineStyle','-.', 'Color','cyan',  'LineWidth',linewidth, 'Parent',ax);
            % legend_labels = [legend_labels, ['$\pm1\sigma$']];
        end
        ylabel(y_ax_labels{i},'interpreter','latex','fontsize',18, 'Parent',ax);
        if (i==1)
            legend(ax, {'target','target estimate',['robot ' type]}, ...
                'interpreter','latex','fontsize',fontsize, 'orientation','horizontal', 'location','northoutside'); 
        end
        axis(ax,'tight');
        hold(ax,'off');
    end

    ax = subplot(4,1,4);
    hold(ax,'on');
    if (~isempty(tau)), plot([Time(1) Time(end)],[tau tau], 'LineStyle','--', 'Color',[0 0 1], 'LineWidth',linewidth, 'Parent',ax); end
    plot(Time,tau_data, 'LineStyle','-', 'Color',[0.85 0.33 0.1], 'LineWidth',linewidth, 'Parent',ax);
    if (plot_1sigma)
        plot(Time,tau_data+Sigma_theta_data(end,:),'c-.', 'LineWidth',linewidth, 'Parent',ax);
        plot(Time,tau_data-Sigma_theta_data(end,:),'c-.', 'LineWidth',linewidth, 'Parent',ax);
        % legend_labels = [legend_labels, ['$\pm1\sigma$']];
    end
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize, 'Parent',ax);
    ylabel('$\tau$ [$s$]','interpreter','latex','fontsize',fontsize, 'Parent',ax);
    if (~isempty(tau)), legend(ax, {'$\tau$','$\hat{\tau}$'},'interpreter','latex','fontsize',fontsize);
    else, legend(ax, {'$\hat{\tau}$'},'interpreter','latex','fontsize',fontsize); end
    axis(ax,'tight');
    hold(ax,'off');

end


function plotSigma(Time, Sigma_theta_data, type)

    figure
    hold on;
    plot(Time, Sigma_theta_data(1,:), 'LineWidth',2.0, 'Color','red');
    plot(Time, Sigma_theta_data(2,:), 'LineWidth',2.0, 'Color','green');
    plot(Time, Sigma_theta_data(3,:), 'LineWidth',2.0, 'Color','blue');
    plot(Time, Sigma_theta_data(4,:), 'LineWidth',2.0, 'Color',[0.93 0.69 0.13]);
    legend({'$x$','$y$','$z$','$\tau$'}, 'interpreter','latex', 'fontsize',15);
    title([type ': $\sigma_{\theta}$'], 'interpreter','latex', 'fontsize',15);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
    hold off;

end


function [effort, mean_fo] = plotEffort(Time, F_data, V_data)

    dt = Time(2)-Time(1);

    effort = 0.0;
    Power = zeros(length(Time),1);
    F_norm = zeros(length(Time),1);
    for i=1:size(F_data,2)
        Power(i) = abs(F_data(:,i)'*V_data(:,i));
        effort = effort + abs(Power(i))*dt;
        F_norm(i) = norm(F_data(:,i));
    end
    mean_fo = mean(F_norm);

    linewidth = 2.5;
    fontsize = 15;
    effort_fig = figure;
    effort_fig_ax = cell(2,1);
    effort_fig_ax{1} = subplot(2,1,1);
    plot(Time, Power, 'LineWidth',linewidth)
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',fontsize);
    ylabel('Power [$W$]', 'interpreter','latex', 'fontsize',fontsize);
    legend(effort_fig_ax{1}, {'DMP+EKF'}, 'interpreter','latex', 'fontsize',fontsize);
    axis tight;

    effort_fig_ax{2} = subplot(2,1,2);
    plot(Time, F_norm, 'LineWidth',linewidth)
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',fontsize);
    ylabel('$||\mathbf{\tau}_{ext}||$ [$Nm$]', 'interpreter','latex', 'fontsize',fontsize);
    axis tight;

end
