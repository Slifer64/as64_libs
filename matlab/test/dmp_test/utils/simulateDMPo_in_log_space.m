function [Time, Q_data, rotVel_data, rotAccel_data] = simulateDMPo_in_log_space(dmp_o, Q0, Qg, T, dt)
%% Simulates a dmp encoding Cartesian orientation usning unit quaternions.


%% set initial values
t_end = T;

iters = 0;
Time = [];
Q_data = [];
rotVel_data = [];
rotAccel_data = [];

t = 0.0;
x = 0.0;
dx = 0.0;
Q = Q0;
Q_prev = Q;
rotVel = zeros(3,1);
rotAccel = zeros(3,1);
q = dmp_o.quat2q(Q0, Q0);
qdot = zeros(3,1);
dy = zeros(3,1);
dz = zeros(3,1);

dmp_o.setTau(t_end);
dmp_o.setQ0(Q0);
y = dmp_o.getY(Q);
z = dmp_o.getZ(rotVel, Q);
g = dmp_o.quat2q(Qg, Q0);

q_data = [];

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    Q_data = [Q_data Q];
    rotVel_data = [rotVel_data rotVel];  
    rotAccel_data = [rotAccel_data rotAccel];
    
    tau_dot = 0;
    yc_dot = 0;
    
    q_data = [q_data q];

    %% DMP simulation
    yc = zeros(3,1);
    zc = zeros(3,1);

    dmp_o.update(x, y, z, g, yc, zc);

    dy = dmp_o.getYdot();
    dz = dmp_o.getZdot();
    rotAccel = dmp_o.getRotAccel(Q, tau_dot, yc_dot);

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

end

