clc;
close all;
clear;

set_matlab_utils_path();


filename = 'recorded_poses.bin';

f_io = FileIO(filename, FileIO.in);

f_io.printHeader();

ee_pos_data = f_io.read('ee_pos_data');
ee_quat_data = f_io.read('ee_quat_data');
marker_pos_data = f_io.read('marker_pos_data');
marker_quat_data = f_io.read('marker_quat_data');


