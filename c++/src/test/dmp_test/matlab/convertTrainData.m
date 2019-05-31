function convertTrainData()

clc;
close all;
clear;

addpath('utils/');

convert_dmp_data();
convert_dmp_pos_data();

end

function convert_dmp_data()

    filename = 'dmp_train_data.bin';

    load('data/train_data.mat', 'Data');

    Time = Data.Time;
    yd_data = Data.Pos(1,:);
    dyd_data = Data.Vel(1,:);
    ddyd_data = Data.Accel(1,:);

    out = fopen(['data/' filename],'w');
    if (out < 0), error(['Failed to create ''' filename '''']); end

    write_mat(Time, out, true);
    write_mat(yd_data, out, true);
    write_mat(dyd_data, out, true);
    write_mat(ddyd_data, out, true);

end

function convert_dmp_pos_data()

    filename = 'dmp_pos_train_data.bin';

    load('data/train_data.mat', 'Data');

    Time = Data.Time;
    Pd_data = Data.Pos;
    dPd_data = Data.Vel;
    ddPd_data = Data.Accel;

    out = fopen(['data/' filename],'w');
    if (out < 0), error(['Failed to create ''' filename '''']); end

    write_mat(Time, out, true);
    write_mat(Pd_data, out, true);
    write_mat(dPd_data, out, true);
    write_mat(ddPd_data, out, true);

end