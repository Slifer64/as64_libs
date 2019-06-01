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


T_end = 12;

H2 = (s-z1) / ( s * (s-p2) );
figure;
rlocus(H2);
H2c = H2 / (1 + H2);
[y2,t2] = step(H2c, T_end);

H3 = (s-z1) * (s + 9.77) / ( s* (s-p1) * (s-p2) );
figure;
rlocus(H3);
H3c = H3 / (1 + H3);
[y3,t3] = step(H3c, T_end);

H3c2 = 18*H3 / (1 + 18*H3);
z3 = zero(H3c2)
p3 = pole(H3c2)

figure
hold on;
plot(t2, y2, 'LineWidth',2.0, 'Color','blue');
plot(t3, y3, 'LineWidth',2.0, 'Color','magenta');
title('Step response', 'interpreter','latex', 'fontsize',16);
legend({'$a =2 $', '$a = 9.77$'}, 'interpreter','latex', 'fontsize',14);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
hold off;
