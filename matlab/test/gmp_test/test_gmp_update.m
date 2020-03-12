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
gmp = GMP(N_kernels, 30, 100, kernels_std_scaling);
tic
x = Timed/Timed(end);
offline_train_mse = gmp.train(GMP_TRAIN.LS, x, Pd_data);
offline_train_mse
toc

% gmp.wsog.plotPsi(x);

p0 = gmp.getYd(0);
pd0 = Pd_data(1);

%% update
kt = 0.5;
ks = 2;
Pgd = Pd_data(end);
P0d = Pd_data(1);
P0 = P0d;
Pg = ks*(Pgd-P0d) + P0;
T = Timed(end)/kt;
Time = Timed/kt;
x = Time / T;
x_dot = 1/T;
x_ddot = 0;

gmp.setGoal(Pg);

N = length(x);
P_data = zeros(1,N);
dP_data = zeros(1,N);
ddP_data = zeros(1,N);


points = getPoint([], [], [], [], [], [], []); 

t1 = 1.5 / kt;
p1 = 0.4;
points(1) = getPoint(t1, t1/T, x_dot, x_ddot, ks*(p1-p0)+p0, [], []);

t2 = 3 / kt;
p2_dot = 0.1;
points(2) = getPoint(t2, t2/T, x_dot, x_ddot, [], ks*p2_dot, []);

t3 = 4.25 / kt;
p3_ddot = 0.3;
points(3) = getPoint(t3, t3/T, x_dot, x_ddot, [], [], ks*p3_ddot);

t4 = 5.5 / kt;
p4 = 0.35;
p4_ddot = 0.2;
points(4) = getPoint(t4, t4/T, x_dot, x_ddot, ks*(p4-p0)+p0, [], ks*p4_ddot);

t5 = 7 / kt;
p5_dot = 0.1;
p5_ddot = 0.0;
points(5) = getPoint(t5, t5/T, x_dot, x_ddot, [], ks*p5_dot, ks*p5_ddot);

t6 = 8.2 / kt;
p6 = 0.4;
p6_dot = 0.5;
p6_ddot = 0.0;
points(6) = getPoint(t6, t6/T, x_dot, x_ddot, ks*(p6-p0)+p0, ks*p6_dot, ks*p6_ddot);


for i=1:length(points), gmp = updateGMP(gmp, points(i)); end

%% simulate
for i=1:N
    P_data(i) = gmp.getYd(x(i));
    dP_data(i) = gmp.getYdDot(x(i), x_dot);
    ddP_data(i) = gmp.getYdDDot(x(i), x_dot, x_ddot);
end


%% obtain scaled demo data
Timed = Timed / kt;
Pd_data = ks*(Pd_data-P0d) + P0;
dPd_data = ks*kt*dPd_data;
ddPd_data = ks*kt^2*ddPd_data;
  

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


for i=1:length(points), plotUpdatePoint(points(i), [ax1 ax2 ax3]); end

%% ==================================================================
%% ==================================================================

function point = getPoint(t, x, x_dot, x_ddot, p, p_dot, p_ddot)
    
    point = struct('t',t, 'x',x, 'x_dot',x_dot, 'x_ddot',x_ddot, 'p',p, 'p_dot',p_dot, 'p_ddot',p_ddot);

end

function gmp = updateGMP(gmp, point)

    if (isempty(point)), return; end

    si = [point.x; point.x_dot; point.x_ddot];
    
    s = [];
    type = [];
    z = [];
    
    if (~isempty(point.p))
        s = [s si];
        z = [z point.p];
        type = [type GMP_UPDATE_TYPE.POS];
    end
    
    if (~isempty(point.p_dot))
        s = [s si];
        z = [z point.p_dot];
        type = [type GMP_UPDATE_TYPE.VEL];
    end
    
    if (~isempty(point.p_ddot))
        s = [s si];
        z = [z point.p_ddot];
        type = [type GMP_UPDATE_TYPE.ACCEL];
    end
    
    gmp.updateWeights(s, z, type);
        
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

