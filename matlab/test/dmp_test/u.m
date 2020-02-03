clc;
close all
clear;


syms z k c

f = collect( (z-6.7)^2 + (4+5*k)*(z-6.7) + 5*k*c , 'z');


