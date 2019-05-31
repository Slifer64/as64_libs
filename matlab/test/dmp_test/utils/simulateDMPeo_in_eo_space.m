function [Time, Q_data, rotVel_data, rotAccel_data] = simulateDMPeo_in_eo_space(dmp_o, Q0, Qg, T, dt)
%% Simulates a dmp encoding Cartesian orientation usning unit quaternions.


%% set initial values
can_clock_ptr = dmp_o.can_clock_ptr;

t_end = T;
can_clock_ptr.setTau(t_end);

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
eo = DMP_eo.quat2eo(Q0, Qg);
deo = zeros(3,1);
dy = zeros(3,1);
dz = zeros(3,1);

dmp_o.setQ0(Q0);
[y, z] = dmp_o.setQg(Qg, Q);


eo_data = [];

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    Q_data = [Q_data Q];
    rotVel_data = [rotVel_data rotVel];  
    rotAccel_data = [rotAccel_data rotAccel];
    rotAccel_data2 = [rotAccel_data2 rotAccel2];
    
    tau_dot = 0;
    yc_dot = 0;
    
    eo_data = [eo_data eo];

    %% DMP simulation
    yc = zeros(3,1);
    zc = zeros(3,1);

    dmp_o.update(x, y, z, yc, zc);

    dy = dmp_o.getYdot();
    dz = dmp_o.getZdot();
    
    rotAccel = dmp_o.getRotAccel(Q, Qg, tau_dot, yc_dot);
    rotAccel2 = dmp_o.calcRotAccel(x, Q, rotVel, Qg, Q0);
    
%     disp('=====================================');
%     rotAccel_err = rotAccel-rotAccel2
%     if (norm(rotAccel_err)>1e-6), pause; end

    %% Update phase variable
    dx = can_clock_ptr.getPhaseDot(x);

    %% Stopping criteria   
    if (t>1.5*t_end)
        warning('Time limit reached... Stopping simulation!');
        break;
    end
    
    eo = DMP_eo.quat2eo(Q, Qg);
    if (t>=t_end && norm(eo)<0.02)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    x = x + dx*dt;
    y = y + dy*dt;
    z = z + dz*dt;
    
    eo = y;
    dy = z/dmp_o.getTau();
    deo = dy;
    
    Q_prev = Q;
    Q = DMP_eo.eo2quat(eo, Qg);
    if (Q_prev'*Q<0), Q = -Q; end
    
    Qe = DMP_eo.quatError(Q, Qg);
    rotVel = DMP_eo.deo2rotVel(deo, Qe);
    
end

eo_data2 = zeros(size(eo_data));
for j=1:size(eo_data2,2)
    eo_data2(:,j) = DMP_eo.quat2eo(Q_data(:,j), Qg);
end

% figure
% plot(Time, eo_data(1,:), Time, eo_data(2,:), Time, eo_data(3,:))
% legend({'e_1','e_2','e_3'});

% figure
% plot(Time, rotAccel_data(1,:), Time, rotAccel_data2(1,:));
% legend({'$\dot{\omega}$', '$\dot{\omega}_2$'}, 'interpreter','latex', 'fontsize',15);
% 
% stop

end

