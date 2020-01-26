clc;
% close all;
clear;

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos(1,:);
dPd_data = Data.Vel(1,:);
ddPd_data = Data.Accel(1,:);

Ts = Timed(2)-Timed(1);

%% initialize and train GMP
N_kernels = 50;
% The scaling affects the smoothness  and update region of the new trajectory.
% Large overlapping between the kernels:
%  + smoother updated trajectory
%  - the locally updated region is larger, so the overall trajectory changes more 
kernels_std_scaling = 2; % good compromise between smoothness and locallity of update
wsog = WSoG(N_kernels, kernels_std_scaling);
tic
x = Timed/Timed(end);
offline_train_mse = wsog.train(DMP_TRAIN.LS, x, Pd_data);
offline_train_mse
toc

% wsog.plotPsi(x);

p0 = wsog.output(0);
pd0 = Pd_data(1);

%% update
temp_s = 0.5;
spat_s = 2;
Pgd = Pd_data(end);
P0d = Pd_data(1);
P0 = P0d;
Pg = spat_s*(Pgd-P0d) + P0;
T = Timed(end)/temp_s;
x_dot = 1/T;
Time = Timed/temp_s;
x = Time / T;
x_ddot = 0;

wsog.setFinalValue(Pg);

N = length(x);
P_data = zeros(1,N);
dP_data = zeros(1,N);
ddP_data = zeros(1,N);


t1 = 1.5 / temp_s;
p1 = 0.4;
s1 = struct('t',t1, 'x',t1/T, 'x_dot',x_dot, 'p',spat_s*(p1-p0)+p0, 'p_dot',[], 'p_ddot',[]);
wsog = updateWSoG(wsog, s1);


t2 = 3 / temp_s;
p2_dot = 0.1;
s2 = struct('t',t2, 'x',t2/T, 'x_dot',x_dot, 'p',[], 'p_dot',spat_s*p2_dot, 'p_ddot',[]);
wsog = updateWSoG(wsog, s2);

t3 = 4.25 / temp_s;
p3_ddot = 0.3;
s3 = struct('t',t3, 'x',t3/T, 'x_dot',x_dot, 'p',[], 'p_dot',[], 'p_ddot',spat_s*p3_ddot);
wsog = updateWSoG(wsog, s3);

t4 = 5.5 / temp_s;
p4 = 0.35;
p4_ddot = 0.2;
s4 = struct('t',t4, 'x',t4/T, 'x_dot',x_dot, 'p',spat_s*(p4-p0)+p0, 'p_dot',[], 'p_ddot',spat_s*p4_ddot);
wsog = updateWSoG(wsog, s4);

t5 = 7 / temp_s;
p5_dot = 0.1;
p5_ddot = 0.0;
s5 = struct('t',t5, 'x',t5/T, 'x_dot',1/T, 'p',[], 'p_dot',spat_s*p5_dot, 'p_ddot',spat_s*p5_ddot);
wsog = updateWSoG(wsog, s5);

t6 = 8.2 / temp_s;
p6 = 0.4;
p6_dot = 0.5;
p6_ddot = 0.0;
s6 = struct('t',t6, 'x',t6/T, 'x_dot',x_dot, 'p',spat_s*(p6-p0)+p0, 'p_dot',spat_s*p6_dot, 'p_ddot',spat_s*p6_ddot);
wsog = updateWSoG(wsog, s6);

%% simulate
for i=1:N
    P_data(i) = wsog.output(x(i));
    dP_data(i) = wsog.outputDot(x(i), x_dot);
    ddP_data(i) = wsog.outputDDot(x(i), x_dot, x_ddot);
end


%% Plot results
fig = figure;
ax1 = subplot(3,1,1);
hold on;
plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
plot(Timed/temp_s, spat_s*(Pd_data-pd0)+pd0, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);

axis tight;
hold off;

ax2 = subplot(3,1,2);
hold on;
plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed/temp_s, spat_s*dPd_data*temp_s, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);

axis tight;
hold off;

ax3 = subplot(3,1,3);
hold on;
plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed/temp_s, spat_s*ddPd_data*temp_s^2, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

plotUpdatePoint(s1, [ax1 ax2 ax3]);
plotUpdatePoint(s2, [ax1 ax2 ax3]);
plotUpdatePoint(s3, [ax1 ax2 ax3]);
plotUpdatePoint(s4, [ax1 ax2 ax3]);
plotUpdatePoint(s5, [ax1 ax2 ax3]);
plotUpdatePoint(s6, [ax1 ax2 ax3]);


%% ==================================================================
%% ==================================================================

function wsog = updateWSoG(wsog, s)

    if (isempty(s)), return; end

    if (~isempty(s.p) && isempty(s.p_dot) && isempty(s.p_ddot))
        wsog.updatePos(s.x, s.p);
    elseif (isempty(s.p) && ~isempty(s.p_dot) && isempty(s.p_ddot))
        wsog.updateVel(s.x, s.x_dot, s.p_dot);
    elseif (isempty(s.p) && isempty(s.p_dot) && ~isempty(s.p_ddot))
        wsog.updateAccel(s.x, s.x_dot, s.p_ddot);
    elseif (~isempty(s.p) && ~isempty(s.p_dot) && isempty(s.p_ddot))
        wsog.updatePosVel(s.x, s.x_dot, s.p, s.p_dot);
    elseif (~isempty(s.p) && isempty(s.p_dot) && ~isempty(s.p_ddot))
        wsog.updatePosAccel(s.x, s.x_dot, s.p, s.p_ddot);
    elseif (isempty(s.p) && ~isempty(s.p_dot) && ~isempty(s.p_ddot))
        wsog.updateVelAccel(s.x, s.x_dot, s.p_dot, s.p_ddot);
    elseif (~isempty(s.p) && ~isempty(s.p_dot) && ~isempty(s.p_ddot))
        wsog.updatePosVelAccel(s.x, s.x_dot, s.p, s.p_dot, s.p_ddot); 
    end
        
end


function plotUpdatePoint(s, ax)

    if (isempty(s)), return; end
      
    if (~isempty(s.p))
        hold(ax(1), 'on');
        scatter([s.t], [s.p], 'MarkerEdgeColor','red', 'LineWidth',2, 'SizeData', 100, 'Parent',ax(1));
        plot([s.t s.t], ax(1).YLim, 'LineWidth',1, 'Color','green', 'LineStyle','--', 'Parent',ax(1));
        hold(ax(1), 'off');
    end
    
    if (~isempty(s.p_dot))
        hold(ax(2), 'on');
        scatter([s.t], [s.p_dot], 'MarkerEdgeColor','red', 'LineWidth',2, 'SizeData', 100, 'Parent',ax(2));
        plot([s.t s.t], ax(2).YLim, 'LineWidth',1, 'Color','green', 'LineStyle','--', 'Parent',ax(2));
        hold(ax(2), 'off');
    end
    
    if (~isempty(s.p_ddot))
        hold(ax(3), 'on');
        scatter([s.t], [s.p_ddot], 'MarkerEdgeColor','red', 'LineWidth',2, 'SizeData', 100, 'Parent',ax(3));
        plot([s.t s.t], ax(3).YLim, 'LineWidth',1, 'Color','green', 'LineStyle','--', 'Parent',ax(3));
        hold(ax(3), 'off');
    end

end