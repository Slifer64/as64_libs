clc;
close all
clear;

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos;
dPd_data = Data.Vel;
ddPd_data = Data.Accel;

Ts = Timed(2)-Timed(1);

%% initialize and train GMP
train_method = 'LS';
N_kernels = 50;
kernels_std_scaling = 2;
n_dof = 3;
gmp = GMP_nDoF(n_dof, N_kernels, 30, 100, kernels_std_scaling);
tic
offline_train_mse = gmp.train(train_method, Timed, Pd_data);
offline_train_mse
toc

% gmp.exportToFile('data/gmp_nDoF_model.bin');
% gmp = GMP_nDoF.importFromFile('data/gmp_nDoF_model.bin');

%% DMP simulation
disp('GMP simulation...');
tic

Time = Timed;
tau = Timed(end);
x_dot = 1/tau;
x = Time / Time(end);
n_data = length(Time);
P_data = zeros(3, n_data);
Vel_dir = zeros(3, n_data);
Mov_dir = zeros(3, n_data);
for j=1:n_data
    P_data(:,j) = gmp.getYd(x(j));
    Vel_dir(:,j) = gmp.getYdDot(x(j), x_dot);
    Vel_dir(:,j) = Vel_dir(:,j) / norm(Vel_dir(:,j));
    Mov_dir(:,j) = gmp.getYdDot(x(j), 1);
    Mov_dir(:,j) = Mov_dir(:,j) / norm(Mov_dir(:,j));
end

for i=1:3
   Mov_dir(i,:) = diff([P_data(i,1) P_data(i,:)]);
end
% Mov_dir = [Mov_dir(:,1) Mov_dir];
for j=1:n_data
    Mov_dir(:,j) = Mov_dir(:,j) / norm(Mov_dir(:,j));
end

%% Plot results
figure;
for i=1:3
    subplot(3,1,i);
    hold on;
    plot(Time, Vel_dir(i,:), 'LineWidth',2.0 , 'Color','blue');
    plot(Timed, Mov_dir(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
    legend({'vel-dir','move-dir'}, 'interpreter','latex', 'fontsize',15);
    axis tight;
    hold off;
end

