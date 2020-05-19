clc;
close all;
clear;

set_matlab_utils_path();

% fid = FileIO('train_data.bin', FileIO.in);
fid = FileIO('gmp_model.bin', FileIO.in);

fid.printHeader();

