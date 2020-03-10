%% Simulates a GMP as a DMP
%  Loads a reference trajecory.
%  Trains a GMP based on the reference trajectory.
%  Plots and compares the results.
function test_gmp_variable_time_scaling()

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos;
dPd_data = Data.Vel;
ddPd_data = Data.Accel;

Ts = Timed(2)-Timed(1);

%% initialize and train GMP
train_method = GMP_TRAIN.LS;
N_kernels = 50;
kernels_std_scaling = 2;
n_dof = 3;
D = 10; % D = 30
K = 50; % K = 100
gmp = GMP_nDoF(n_dof, N_kernels, D, K, kernels_std_scaling);
tic
offline_train_mse = gmp.train(train_method, Timed, Pd_data);
offline_train_mse
toc

%% Compare with the learned trajectory
% [Timed, Pd_data, dPd_data, ddPd_data] = simulateGMP_nDoF(gmp, Pd_data(:,1), Pd_data(:,end), Timed(end), Ts);

%% DMP simulation
disp('GMP simulation...');
tic

spat_s = 1.5; % spatial scale
temp_s = 1.5; % temporal scale
P0 = Pd_data(:,1);
Pgd = Pd_data(:,end);
Pg = P0 + spat_s*(Pgd - P0);
T = Timed(end) / temp_s;
dt = 0.002; %Ts;

%% set initial values
Dim = gmp.length();
y_ddot = zeros(Dim,1);
y_dot = zeros(Dim,1);
y0 = P0;
g = Pg;
y = y0;
t = 0.0;
z_dot = zeros(Dim,1);
z = zeros(Dim,1);

t_end = T;
tau = t_end;
x = 0.0;
x_dot = 1/tau;
xd_dot = 1/tau;
x_ddot = 0;
s = [x; x_dot; x_ddot];
fv_prev = 0;

iters = 0;
Time = [];
P_data = [];
dP_data = [];
ddP_data = [];
x_data = [];
x_dot_data = [];
zc_data = [];
z_c = zeros(3,1);

gmp.setY0(y0);
gmp.setGoal(g);

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    P_data = [P_data y];
    dP_data = [dP_data y_dot];  
    ddP_data = [ddP_data y_ddot];
    x_data = [x_data x];
    x_dot_data = [x_dot_data x_dot];
    zc_data = [zc_data z_c];

    %% DMP simulation
    y_c = 0.0;
    z_c = 0.0;
    fdist_x = disturbance_function(t, t_end, 10); 
    fdist_y = disturbance_function(t, t_end, -8);
    fdist_z = disturbance_function(t, t_end, -9);
    z_c = [fdist_x; fdist_y; fdist_z];
    gmp.update(s, y, z, y_c, z_c);
    y_dot = gmp.getYdot();
    z_dot = gmp.getZdot();

    yc_dot = 0.0;
    y_ddot = gmp.getYddot(yc_dot);

    %% Update phase variable
    if (t < 0.1*t_end), fv = 5;
    elseif (t < 0.2*t_end), fv = 20;
    elseif (t < 0.3*t_end), fv = 0;
    elseif (t < 0.5*t_end), fv = -25;
    elseif (t < 0.7*t_end), fv = 10;
    elseif (t < 0.9*t_end), fv = -25;
    else, fv = 10;
    end
    
    a_v = 0.99;
    fv = a_v*fv_prev + (1-a_v)*fv;
    fv_prev = fv;
    x_ddot = 50*(xd_dot - x_dot) + fv;
    % xd_ddot = 50*(xd_dot - x_dot) + fv;
    % x_3dot = 20*(xd_ddot - x_ddot);
    % x_3dot = 200*(xd_ddot - x_ddot);

    %% Stopping criteria
    if (x>=1.1) % && norm(y-g)<5e-3 && norm(dy)<5e-3)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    x = x + x_dot*dt;
    x_dot = x_dot + x_ddot*dt;
    %x_ddot = x_ddot + x_3dot*dt;
    y = y + y_dot*dt;
    z = z + z_dot*dt;

    s = [x; x_dot; x_ddot];
%     s = [x; x_dot; 0*x_ddot];
    
end

toc


%% Reference trajectory (scaled)
Timed = Timed / temp_s;
Pd_data = spat_s*( Pd_data-P0 ) + P0;
dPd_data = spat_s*dPd_data*temp_s;
ddPd_data = spat_s*ddPd_data*temp_s^2;


for i=1:3
    [dist, ix] = fb_dtw(P_data(i,:), Pd_data(i,:));
end

Time2 = cell(3,1);
P_data2 = cell(3,1);
Pd_data2 = cell(3,1);
dist = zeros(3,1);
n_w = 0;
for i=1:3
    [dist(i), ix] = fb_dtw(P_data(i,:), Pd_data(i,:));
    Pd_data2{i} = Pd_data(i,ix);
    P_data2{i} = P_data(i,:);
    n_w = length(ix);
    Time2{i} = (0:(n_w-1))*Ts;
    dist(i) = dist(i) / n_w;
end
dist


% Time2 = cell(3,1);
% P_data2 = cell(3,1);
% Pd_data2 = cell(3,1);
% dist = zeros(3,1);
% n_w = 0;
% for i=1:3
%     [dist(i), ix, iy] = dtw(Pd_data(i,:), P_data(i,:));
%     Pd_data2{i} = Pd_data(i,ix);
%     P_data2{i} = P_data(i,iy);
%     n_w = length(ix);
%     Time2{i} = (0:(n_w-1))*Ts;
%     dist(i) = dist(i) / n_w;
% end
% dist

figure;
for i=1:3
subplot(3,2, (i-1)*2+1);
hold on;
plot(Time2{i}, Pd_data2{i}, 'LineWidth',2, 'Color','blue', 'LineStyle','-');
plot(Time2{i}, P_data2{i}, 'LineWidth',2, 'Color','magenta', 'LineStyle',':');
hold off;
subplot(3,2, (i-1)*2+2);
plot(Time2{i}, P_data2{i}-Pd_data2{i}, 'LineWidth',2, 'Color','red');
end


%% Plot results
% for i=1:3
%     figure;
%     subplot(3,1,1);
%     hold on;
%     plot(Time, P_data(i,:), 'LineWidth',2.0 , 'Color','blue');
%     plot(Timed, Pd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
%     ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
%     title(['temporal scale: $' num2str(temp_s) '$     ,     spatial scale: $' num2str(spat_s) '$'], 'interpreter','latex', 'fontsize',18);
%     legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);
% 
%     axis tight;
%     hold off;
% 
%     subplot(3,1,2);
%     hold on;
%     plot(Time, dP_data(i,:), 'LineWidth',2.0, 'Color','blue');
%     plot(Timed, dPd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
%     ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
%     axis tight;
%     hold off;
% 
%     subplot(3,1,3);
%     hold on;
%     plot(Time, ddP_data(i,:), 'LineWidth',2.0, 'Color','blue');
%     plot(Timed, ddPd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
%     ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
%     axis tight;
%     hold off;
% end


plot_animated = true;
if (plot_animated)
    
    figure;
    ax = subplot(2,1,1);
    hold(ax,'on');
    x_pl = plot(nan, nan, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1], 'Parent',ax);
    plot([Time(1) Time(end)], [0 Time(end)/tau], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
    ylabel('$x$', 'interpreter','latex', 'fontsize',15);
    ax.XLim = [Time(1) Time(end)];
    ax.YLim = [0 1.2];
    hold(ax,'off');
    ax = subplot(2,1,2);
    hold(ax,'on');
    xdot_pl = plot(nan, nan, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1], 'Parent',ax);
    plot([Time(1) Time(end)], [1/tau 1/tau], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
    ylabel('$\dot{x}$', 'interpreter','latex', 'fontsize',15);
    ax.XLim = [Time(1) Time(end)];
    ax.YLim = [min(x_dot_data) max(x_dot_data)] + 0.15*[-1 1];
    hold(ax,'off');
    
    %% animated 3D path
    ax = axes('Parent',figure());
    hold(ax,'on');
    plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue', 'Parent',ax);
    plot_animated = true;
    if (plot_animated)
        pl = plot3(nan, nan, nan, 'LineWidth',4, 'LineStyle','-', 'Marker','o', 'MarkerSize',10, 'Color','red', 'Parent',ax);
        zc_quiv = quiver3(nan,nan,nan, nan,nan,nan, 'Parent',ax, ...
            'LineWidth',4, 'LineStyle','-', 'MarkerFaceColor',[0 0.8 0], 'MarkerSize',5, 'MaxHeadSize',0.9, 'AutoScaleFactor',0.02);
        for j=1:size(P_data,2)
            % pl.XData = [pl.XData P_data(1,j)];
            % pl.YData = [pl.YData P_data(2,j)];
            % pl.ZData = [pl.ZData P_data(3,j)]; 
            pl.XData = [P_data(1,j)];
            pl.YData = [P_data(2,j)];
            pl.ZData = [P_data(3,j)];
            zc_quiv.XData = P_data(1,j); zc_quiv.YData = P_data(2,j); zc_quiv.ZData = P_data(3,j);
            zc_quiv.UData = zc_data(1,j); zc_quiv.VData = zc_data(2,j); zc_quiv.WData = zc_data(3,j);

            ax.XLim = [0 0.6000];
            ax.YLim = [0 1.2000];
            % ax.ZLim = z_lim;
            
            x_pl.XData = [x_pl.XData Time(j)];
            x_pl.YData = [x_pl.YData x_data(j)];
            xdot_pl.XData = [xdot_pl.XData Time(j)];
            xdot_pl.YData = [xdot_pl.YData x_dot_data(j)];

            pause(0.01);
            drawnow
        end

    end
    plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta', 'Parent',ax);
    hold(ax,'off');
    
else
   
    figure;
    subplot(2,1,1);
    hold on;
    plot(Time, x_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
    plot([Time(1) Time(end)], [0 Time(end)/tau], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
    ylabel('$x$', 'interpreter','latex', 'fontsize',15);
    hold off;
    subplot(2,1,2);
    hold on;
    plot(Time, x_dot_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
    plot([Time(1) Time(end)], [1/tau 1/tau], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
    ylabel('$\dot{x}$', 'interpreter','latex', 'fontsize',15);
    hold off;
    
    ax = axes('Parent',figure());
    hold(ax,'on');
    plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue', 'Parent',ax);
    plot3(P_data(1,end), P_data(2,end), P_data(3,end), 'LineWidth',4, 'LineStyle','-', 'Marker','o', 'MarkerSize',10, 'Color','red', 'Parent',ax);
    plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta', 'Parent',ax);
    hold(ax,'off');

end



end


function fdist = disturbance_function(t, T, f_max)
    
    t1 = 0.4* 0.45*T;
    t2 = 0.4* 0.55*T;
    t3 = 0.4* 0.75*T;
    t4 = 0.4* 0.85*T;

    if (t < t1), fdist = 0;
    elseif (t < t2), fdist = f_max*(t - t1)/(t2-t1);
    elseif (t < t3), fdist = f_max;
    elseif (t < t4), fdist = f_max - f_max*(t - t3)/(t4-t3);
    else, fdist = 0;
    end

end


function [dist, ix] = fb_dtw(x, y)

n1 = length(x);
dist = 0;

ix = zeros(size(x));
for i=1:n1
    [min_dist, ix(i)] = min(abs(y-x(i)));  
    dist = dist + min_dist;
end


end
