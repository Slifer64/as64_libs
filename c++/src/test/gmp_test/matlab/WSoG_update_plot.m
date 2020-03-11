addpath('utils/');

filename = 'data/wsog_update_sim_data.bin';
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

n_points = read_scalar(in, true, 'uint64');
s = cell(n_points,1);
for i=1:n_points
   a = read_mat(in, true);
   s{i} = struct( 't',a(1), 'x',a(2), 'x_dot',a(3), 'x_ddot',a(4), ...
       'p',a(5), 'p_dot',a(6), 'p_ddot',a(7), ...
       'update_pos',a(8), 'update_vel',a(9), 'update_accel',a(10) );
end
fclose(in);


%% Plot results
fig = figure;
ax1 = subplot(3,1,1);
hold on;
plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
plot(Timed, Pd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);

axis tight;
hold off;

ax2 = subplot(3,1,2);
hold on;
plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, dPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);

axis tight;
hold off;

ax3 = subplot(3,1,3);
hold on;
plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, ddPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

for i=1:length(s), plotUpdatePoint(s{i}, [ax1 ax2 ax3]); end


%% ==================================================================
%% ==================================================================


function plotUpdatePoint(s, ax)

    if (s.update_pos)
        hold(ax(1), 'on');
        scatter([s.t], [s.p], 'MarkerEdgeColor','red', 'LineWidth',2, 'SizeData', 100, 'Parent',ax(1));
        plot([s.t s.t], ax(1).YLim, 'LineWidth',1, 'Color','green', 'LineStyle','--', 'Parent',ax(1));
        hold(ax(1), 'off');
    end
    
    if (s.update_vel)
        hold(ax(2), 'on');
        scatter([s.t], [s.p_dot], 'MarkerEdgeColor','red', 'LineWidth',2, 'SizeData', 100, 'Parent',ax(2));
        plot([s.t s.t], ax(2).YLim, 'LineWidth',1, 'Color','green', 'LineStyle','--', 'Parent',ax(2));
        hold(ax(2), 'off');
    end
    
    if (s.update_accel)
        hold(ax(3), 'on');
        scatter([s.t], [s.p_ddot], 'MarkerEdgeColor','red', 'LineWidth',2, 'SizeData', 100, 'Parent',ax(3));
        plot([s.t s.t], ax(3).YLim, 'LineWidth',1, 'Color','green', 'LineStyle','--', 'Parent',ax(3));
        hold(ax(3), 'off');
    end

end