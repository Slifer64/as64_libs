clc;
close all;
clear;

addpath('utils/');

filename = '../data/logged_data.bin';
fid = fopen(filename);
if (fid < 0), error('Faile to open %s\n', filename); end

Time = read_mat(fid);
Time = (Time - Time(1))/1000;
q_data = read_mat(fid);
dq_data = read_mat(fid);
% dq_target_data = read_mat(fid);
q_cmd_data = read_mat(fid);
dq_cmd_data = read_mat(fid);


%% ========== position ==============
for i=1:2

    figure;
    hold on;
    plot(q_data(i,:), 'LineWidth',1.5, 'Color','blue');
    plot(q_cmd_data(i,:), 'LineWidth',1.5, 'Color','magenta');
    hold off;
    
end


%% ========== velocity =============

for i=1:2

    figure;
    hold on;
    plot(Time, dq_data(i,:), 'LineWidth',1.5, 'Color','blue');
%     plot(dq_target_data(i,:), 'LineWidth',1.5, 'Color','cyan');
    plot(Time, dq_cmd_data(i,:), 'LineWidth',1.5, 'Color','magenta');
    hold off;
    
end

%% ========== ctrl cycle =============

Ts = diff(Time);
figure;
plot(Ts)
mean(Ts)
std(Ts)
