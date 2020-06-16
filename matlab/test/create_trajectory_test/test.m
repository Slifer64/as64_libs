clc;
close all;
clear;

set_matlab_utils_path();

ax = axes();
ax.XLim = [0 1];
ax.YLim = [0 1];

cr_traj = CreateTraj_Splines(ax);
% cr_traj = CreateTraj_WSoG(ax);
% cr_traj = CreateTraj_Poly(ax);
      