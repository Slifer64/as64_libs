function [Time, P_data, dP_data, ddP_data] = trimData(Time, P_data, dP_data, ddP_data, vel_tol)

n = length(Time);

for j=1:length(Time)
   if (norm(dP_data(:,j)) > vel_tol), break; end
end
i1 = j;

for j=n:-1:i1
   if (norm(dP_data(:,j)) > vel_tol), break; end
end
i2 = j;

Time = Time(i1:i2) - Time(i1);
P_data = P_data(i1:i2);
dP_data = dP_data(i1:i2);
ddP_data = ddP_data(i1:i2);

end