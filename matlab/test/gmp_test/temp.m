clc;
close all;
clear;


Q = [ 0.685; 0.725; 0.044; -0.054 ];
Q = Q / norm(Q);

Q2 = rotm2quat(rotz(40))';

Q_ = quatProd(Q, Q2)'


