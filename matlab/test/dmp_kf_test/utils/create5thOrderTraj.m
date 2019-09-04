function Data = create5thOrderTraj(p0, pg, Q0, Qg, T, dt)

Data = struct('Time',[], 'Pos',[], 'Vel',[], 'Accel',[], 'ddPd_data',[], 'Quat',[], 'RotVel',[], 'RotAccel',[]);

Time = 0:dt:T;
n = length(Time);
Pos = zeros(3,n);
Vel = zeros(3,n);
Accel = zeros(3,n);
Vel2 = zeros(3,n);
Accel2 = zeros(3,n);

Eo = zeros(3,n);
dEo = zeros(3,n);
ddEo = zeros(3,n);
eo0 = quatLog(quatDiff(Qg,Q0));
for i=1:3
    [Pos(i,:), Vel(i,:), Accel(i,:)] = get5th1D(p0(i), pg(i), Time);
    [Eo(i,:), dEo(i,:), ddEo(i,:)] = get5th1D(eo0(i), 0.0, Time);
    
    Vel2(i,:) = [diff(Pos(i,:)) 0] / dt;
    Accel2(i,:) = [diff(Vel2(i,:)) 0] / dt;
end

Quat = zeros(4,n);
RotVel = zeros(3,n);
RotAccel = zeros(3,n);

RotVel2 = zeros(3,n);
RotAccel2 = zeros(3,n);
for j=1:n
    Quat(:,j) = DMP_eo.eo2quat(Eo(:,j), Qg);
    Qe = quatDiff(Qg, Quat(:,j));
    RotVel(:,j) = DMP_eo.deo2rotVel(dEo(:,j), Qe);
    RotAccel(:,j) = DMP_eo.ddeo2rotAccel(ddEo(:,j), RotVel(:,j), Qe);
end

Qdot = zeros(4,n);
for i=1:4
    Qdot(i,:) = [diff(Quat(i,:)) 0] / dt;
end
for j=1:n
    vRot2 = 2*quatProd(Qdot(:,j), quatInv(Quat(:,j)));
    RotVel2(:,j) = vRot2(2:4);
end
for i=1:3
    RotAccel2(i,:) = [diff(RotVel(i,:)) 0] / dt;
end

Data.Time = Time;
Data.Pos = Pos;
Data.Vel = Vel;
Data.Accel = Accel;
Data.Quat = Quat;
Data.RotVel = RotVel;
Data.RotAccel = RotAccel;

figure;
for i=1:3
   subplot(3,1,i);
   plot(Time, Pos(i,:), 'LineWidth',2.0);
   if (i==1), title('Position'); end
   if (i==3), xlabel('time [s]'); end
end

figure;
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, Vel(i,:), 'LineWidth',2.0);
   plot(Time, Vel2(i,:), 'LineWidth',2.0, 'Color','magenta','LineStyle','--');
   if (i==1), title('Velocity'); end
   if (i==3), xlabel('time [s]'); end
   hold off;
end

figure;
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, Accel(i,:), 'LineWidth',2.0);
   plot(Time, Accel2(i,:), 'LineWidth',2.0, 'Color','magenta','LineStyle','--');
   if (i==1), title('Acceleration'); end
   if (i==3), xlabel('time [s]'); end
   hold off;
end

figure;
for i=1:4
   subplot(4,1,i);
   plot(Time, Quat(i,:), 'LineWidth',2.0);
   if (i==1), title('Orientation'); end
   if (i==3), xlabel('time [s]'); end
end

figure;
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, RotVel(i,:), 'LineWidth',2.0);
   plot(Time, RotVel2(i,:), 'LineWidth',2.0, 'Color','magenta','LineStyle','--');
   if (i==1), title('Rotational Velocity'); end
   if (i==3), xlabel('time [s]'); end
   hold off;
end

figure;
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, RotAccel(i,:), 'LineWidth',2.0);
   plot(Time, RotAccel2(i,:), 'LineWidth',2.0, 'Color','magenta','LineStyle','--');
   if (i==1), title('Rotational Acceleration'); end
   if (i==3), xlabel('time [s]'); end
   hold off;
end


end


function [y, dy, ddy] = get5th1D(y0, yf, Time)

T = Time(end);
t = Time/T;

y = y0 + (yf - y0) * (10*t.^3 - 15*t.^4 + 6*t.^5 );

if (nargout > 1)
    dy = (yf - y0) * (30*t.^2 - 60*t.^3 + 30*t.^4 ) / T;
end

if (nargout > 2)
    ddy = (yf - y0) * (60*t - 180*t.^2 + 120*t.^3 ) / T^2;
end

end
