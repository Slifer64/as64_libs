clc;
close all;
clear;

rng(0);


set_matlab_utils_path();


dt = 0.005;
tf = 4.5;
Time = 0:dt:tf;

q0 = [-0.43, 0.96, 0.61, -1.53, -1.66, -0.32, -1.39]';
qf = [0.15, 0.58, 1.5, -1.4, 1.3, 1.19, 0.24]';

N_joints = length(q0);
n_data = length(Time);


%% Create desired trajectory
qd_data = get5thOrderPol(q0, qf, Time);


%% Train ProMP and use it to create many demos
x = Time / Time(end);
N_kernels = 5;
n_samples = 10;
mp = cell(N_joints);
q_data = cell(n_samples);
for i=1:N_joints
    mp{i} = ProMP(N_kernels);
    mp{i}.train(x, qd_data(i,:));
    mp{i}.setPhiBasedSigma(0.05);
end

for k=1:n_samples
    q_data{k} = zeros(N_joints, n_data);
    for i=1:N_joints
        mp{i}.sampleWeights();
        y = zeros(1,n_data);
        for j=1:n_data, y(j) =  mp{i}.output(x(j)); end
        q_data{k}(i,:) = y;
    end
end


%% Calculate Cartesian trajectory
Pd_data = zeros(3, n_data);
Qd_data = zeros(4, n_data);
Q_prev = [1 0 0 0]';
for j=1:n_data
    [Pd_data(:,j), Qd_data(:,j)] = lwr4pPose(qd_data(:,j), Q_prev); 
    Q_prev = Qd_data(:,j);
end

P_data = cell(1, n_samples);
Q_data = cell(1, n_samples);
for k=1:n_samples
    P_data{k} = zeros(3, n_data);
    Q_data{k} = zeros(4, n_data);
    Q_prev = [1 0 0 0]';
    for j=1:n_data
        [P_data{k}(:,j), Q_data{k}(:,j)] = lwr4pPose(q_data{k}(:,j), Q_prev);
        Q_prev = Q_data{k}(:,j);
    end
end

%% Save data
Data = struct('Time',Time, 'qd_data',q_data);
save('data/demo_data.mat', 'Data');


%% Plot Joints trajectory
figure;
for i=1:N_joints
    subplot(N_joints,1,i); hold on;
    for k=1:n_samples, plot(Time, q_data{k}(i,:), 'LineWidth',2, 'Color',[1 0 1 0.3]); end
    plot(Time, qd_data(i,:), 'LineWidth',2, 'Color','blue', 'LineStyle','--');
    ylabel(['j$' num2str(i) '$'], 'interpreter','latex', 'fontsize',15);
    % legend({'$y$','$y_d$'}, 'interpreter','latex', 'fontsize',15);
    if (i==N_joints), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
    if (i==1), title('Joints Position', 'interpreter','latex', 'fontsize',18); end
    axis tight;
end

%% Plot Cartesian trajectory

% position
figure;
for i=1:3
    subplot(3,1,i); hold on;
    for k=1:n_samples, plot(Time, P_data{k}(i,:), 'LineWidth',2, 'Color',[1 0 1 0.3]); end
    plot(Time, Pd_data(i,:), 'LineWidth',2, 'Color','blue', 'LineStyle','--');
    ylabel(['pos-$' num2str(i) '$'], 'interpreter','latex', 'fontsize',15);
    if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
    if (i==1), title('Cartesian Position', 'interpreter','latex', 'fontsize',18); end
    axis tight;
end

% Orientation
figure;
for i=1:4
    subplot(4,1,i); hold on;
    for k=1:n_samples, plot(Time, Q_data{k}(i,:), 'LineWidth',2, 'Color',[1 0 1 0.3]); end
    plot(Time, Qd_data(i,:), 'LineWidth',2, 'Color','blue', 'LineStyle','--');
    ylabel(['q-$' num2str(i) '$'], 'interpreter','latex', 'fontsize',15);
    if (i==4), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
    if (i==1), title('Cartesian Orientation (Quaternion)', 'interpreter','latex', 'fontsize',18); end
    axis tight;
end

%% Cartesian path

% position
figure;
hold on;
for k=1:n_samples
   plot3(P_data{k}(1,:), P_data{k}(2,:), P_data{k}(3,:), 'LineWidth',2, 'Color',[1 0 1 0.3]); 
end
plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth',2, 'Color','blue', 'LineStyle','--');
xlabel('$X$', 'interpreter','latex', 'fontsize',15);
ylabel('$Y$', 'interpreter','latex', 'fontsize',15);
zlabel('$Z$', 'interpreter','latex', 'fontsize',15);

% orientation
eod_data = zeros(3, n_data);
Q0 = Qd_data(:,1);
for j=1:n_data
   eod_data(:,j) = math_.quatLog( math_.quatProd( Qd_data(:,j), math_.quatInv(Q0) ) ); 
end

eo_data = cell(n_samples, 1);
for k=1:n_samples
    Q0 = Q_data{k}(:,1);
    for j=1:n_data
       eo_data{k}(:,j) = math_.quatLog( math_.quatProd( Q_data{k}(:,j), math_.quatInv(Q0) ) ); 
    end
end

figure;
hold on;
for k=1:n_samples
   plot3(eo_data{k}(1,:), eo_data{k}(2,:), eo_data{k}(3,:), 'LineWidth',2, 'Color',[1 0 1 0.3]); 
end
plot3(eod_data(1,:), eod_data(2,:), eod_data(3,:), 'LineWidth',2, 'Color','blue', 'LineStyle','--');
title('Cartesian Orientation path', 'interpreter','latex', 'fontsize',18);
xlabel('$X$', 'interpreter','latex', 'fontsize',15);
ylabel('$Y$', 'interpreter','latex', 'fontsize',15);
zlabel('$Z$', 'interpreter','latex', 'fontsize',15);


