clc;
close all;
clear;

rng(0);

n = 5;

v = rand(n,1);
A = eye(n,n) + v*v';
L = chol(A, 'lower');

Oij = zeros(n,n);

i = 3;
j = 2;
Oij(i,j) = 1;

L

Lij = Oij*L' + (Oij*L')'

B = zeros(n,n);
B(i,:) = L(:,j)';

C = zeros(n,n);
C(:,i) = L(:,j);


Lij_hat = B + C