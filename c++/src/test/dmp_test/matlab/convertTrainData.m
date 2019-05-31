clc;
close all;
clear;

addpath('utils/');

filename = 'dmp_std_train_data.bin';

load('data/train_data.mat', 'Data');

Time = Data.Time;
yd_data = Data.Pos(1,:);
dyd_data = Data.Vel(1,:);
ddyd_data = Data.Accel(1,:);

out = fopen(['data/' filename],'w');
if (out < 0), error(['Failed to create ''' filename '''']); end

write_mat(Time, out, true);
write_mat(yd_data, out, true);
write_mat(dyd_data, out, true);
write_mat(ddyd_data, out, true);
