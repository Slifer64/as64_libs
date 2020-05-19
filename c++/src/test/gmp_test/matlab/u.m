clc;
close all;
clear;


% fid = FileIO('data/gmp_model.bin', FileIO.in);
fid = FileIO('data/gmp_nDoF_model.bin', FileIO.in);

fid.printHeader();