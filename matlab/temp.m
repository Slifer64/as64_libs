clc;
close all;
clear;


fun = @(x) - ( 0.5*x(1)^2 + 0.25*x(3)*x(2)^4 );

x0 = [0; 0; 4.5];


lb = [-1; -1; 1];
ub = [1; 1; 9];

[x,fval] = fmincon(fun,x0, [],[], [],[], lb,ub);

x
-fval

