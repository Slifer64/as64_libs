function [Time, Q_data, rotVel_data, rotAccel_data] = simulateDMPo_in_log_space(dmp_o, Q0, Qg, T, dt)
%% Simulates a dmp encoding Cartesian orientation usning unit quaternions.


%% set initial values
t_end = T;

iters = 0;
Time = [];
Q_data = [];
rotVel_data = [];
rotAccel_data = [];
rotAccel_data2 = [];

t = 0.0;
x = 0.0;
dx = 0.0;
Q = Q0;
Q_prev = Q;
rotVel = zeros(3,1);
rotAccel = zeros(3,1);
rotAccel2 = zeros(3,1);
q = dmp_o.quat2q(Q0, Q0);
qdot = zeros(3,1);
dy = zeros(3,1);
dz = zeros(3,1);

dmp_o.setTau(t_end);
dmp_o.setQ0(Q0);
y = dmp_o.getY(Q);
z = dmp_o.getZ(rotVel, Q);
g = dmp_o.quat2q(Qg, Q0);


%% simulate
while (true)

    %% data logging
    Time = [Time t];
    Q_data = [Q_data Q];
    rotVel_data = [rotVel_data rotVel];  
    rotAccel_data = [rotAccel_data rotAccel];
%     rotAccel_data2 = [rotAccel_data2 rotAccel2];
    
    tau_dot = 0;
    yc_dot = 0;

    %% DMP simulation
    yc = zeros(3,1);
    zc = zeros(3,1);

    dmp_o.update(x, y, z, g, yc, zc);

    dy = dmp_o.getYdot();
    dz = dmp_o.getZdot();
    rotAccel = dmp_o.getRotAccel(Q, tau_dot, yc_dot);
%     rotAccel2 = dmp_o.calcRotAccel(x, Q, rotVel, Qg);

    %% Update phase variable
    dx = dmp_o.phaseDot(x);

    %% Stopping criteria   
    if (t>1.5*t_end)
        warning('Time limit reached... Stopping simulation!');
        break;
    end
    
    eo = quatLog(quatProd(Qg, quatInv(Q)));
    if (t>=t_end && norm(eo)<0.02)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    x = x + dx*dt;
    y = y + dy*dt;
    z = z + dz*dt;
    
    q = y;
    dy = z/dmp_o.getTau();
    qdot = dy;
    
    Q_prev = Q;
    Q = dmp_o.q2quat(q, Q0);
    if (Q_prev'*Q<0), Q = -Q; end
    
    Q1 = dmp_o.quatTf(Q, Q0);
    rotVel = dmp_o.qdot2rotVel(qdot, Q1);
    
end

% n = length(Time);
% Q1_data = zeros(4,n);
% q_data = zeros(3,n);
% 
% for j=1:n
%    Q1_data(:, j) = DMPo.quatTf(Q_data(:,j), Q0);
%    q_data(:, j) = DMPo.quat2q(Q_data(:,j), Q0);
% end
% 
% figure;
% for i=1:3
%    subplot(3,1,i);
%    hold on;
%    plot(Time, rotAccel_data(i,:), 'LineWidth',2, 'Color','blue');
%    plot(Time, rotAccel_data2(i,:), 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
%    if (i==1)
%        title('Rot Accel: $\dot{\omega}$', 'interpreter','latex', 'fontsize',17);
%        legend({'get','calc'}, 'interpreter','latex', 'fontsize',15);
%    end
%    hold off;
% end
% 
% figure;
% for i=1:4
%    subplot(4,1,i);
%    hold on;
%    plot(Time, Q1_data(i,:), 'LineWidth',2, 'Color','blue');
%    if (i==1)
%        title('$Q_1 = Q*\mathbf{Q}_0$', 'interpreter','latex', 'fontsize',17);
%    end
%    hold off;
% end
% 
% error('stop')

end

