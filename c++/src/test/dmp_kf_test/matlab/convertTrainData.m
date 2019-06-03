function convertTrainData()

clc;
close all;
clear;

path = strrep(mfilename('fullpath'), 'convertTrainData','');
addpath([path 'utils/']);

filename = 'train_data.bin';

load([path 'data/train_data.mat'], 'Data');

Time = Data.Time;
Pd_data = Data.Pos;
dPd_data = Data.Vel;
ddPd_data = Data.Accel;
Qd_data = Data.Quat;
vRotd_data = Data.RotVel;
dvRotd_data = Data.RotAccel;

out = fopen([path 'data/' filename],'w');
if (out < 0), error(['Failed to create ''' filename '''']); end

write_mat(Time, out, true);
write_mat(Pd_data, out, true);
write_mat(dPd_data, out, true);
write_mat(ddPd_data, out, true);
write_mat(Qd_data, out, true);
write_mat(vRotd_data, out, true);
write_mat(dvRotd_data, out, true);

end



