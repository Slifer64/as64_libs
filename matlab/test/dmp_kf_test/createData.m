clc;
close all;
clear;

format compact;
set_matlab_utils_path();

T = 5;
dt = 0.005;
p0 = [0 0 0];
pg = [0.8 0.7 1.2];
Q0 = [1 0 0 0]';
Qg = rotm2quat(rotx(50)*rotz(110)*roty(70))';
Data = create5thOrderTraj(p0, pg, Q0, Qg, T, dt);

path = strrep(mfilename('fullpath'), 'createData','');
save([path 'data/train_5traj_data.mat'], 'Data');