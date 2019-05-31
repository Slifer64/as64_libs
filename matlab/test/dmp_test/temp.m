clc;
close all;
clear;

s = tf('s');

z1 = -5;
p1 = -2;
p2 = -10;

Hp = (s-z1) / ( (s-p1) * (s-p2) );

k = 1;
a = 8;
b = 0;
Gc = k * (s + a) / (s + b);

Ho = Gc * Hp;

% figure;
% rlocus(Hp);

figure;
rlocus(Ho);

k = 18;
A = (k*(s-z1)) / (s^3 + (k-p1-p2)*s^2 + (p1*p2-z1*k)*s);
figure;
rlocus(A);

zeros = roots(k*[1 -z1])

poles = roots([1 (k-p1-p2) (p1*p2-z1*k) 0])

sigma_a = ( sum(poles) - sum(zeros) ) / ( length(poles) - length(zeros) )

syms x real;
f1 = k*x - z1*k;
f2 = x^3 + (k-p1-p2)*x^2 + (p1*p2-z1*k)*x;
f = simplify( diff(f1, x)*f2 - f1*diff(f2, x) );
s_i = roots( sym2poly(f)  )

