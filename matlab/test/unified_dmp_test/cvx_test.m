clc;
close all
clear;

cvx_begin sdp
    variable S(n,n) hermitian toeplitz
    minimize( norm( S - Sw, 'fro' ) )
    S >= 0
cvx_end



