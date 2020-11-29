clc;
close all;
clear;

set_matlab_utils_path();
data_path = '../data/';

%% Load training data
load([data_path 'train_data1.mat'], 'Data');

Time = Data.Time;
Pd_data = Data.Pos(1,:);
dPd_data = Data.Vel(1,:);
ddPd_data = Data.Accel(1,:);

[Time, Pd_data, dPd_data, ddPd_data] = trimData(Time, Pd_data, dPd_data, ddPd_data, 5e-3);

Ts = Time(2)-Time(1);

n_data = length(Time);

tau = Time(end);
x = Time / tau;
x0 = 0;
xf = 1;

%% initialize and train GMP
N_kernels = 15;
kernels_std_scaling = 1;
mp = MP(N_kernels, kernels_std_scaling);
tic
offline_train_mse = mp.train(x, Pd_data);
offline_train_mse
toc




mp.optWeightsCovariance();

is_spd = isSPD(mp.Sigma_w)

save('data.mat', 'Time','Pd_data','mp');

