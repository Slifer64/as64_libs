clc;
close all;
clear;

set_matlab_utils_path();


read_data_from_cpp = false;

filename = 'data.bin';


%% Generate some data
b = int8( 1 );
i = int32( -5 );
u = uint32( 16 );
f = single( 5.2 );
d = double( 6.7 );
d_mat = double( [1.1 1.2 1.3; 1.4 1.5 1.6] );
f_vec = single( [3.1 3.2 3.3 3.4 3.5]' );
i_rowvec = int32( [5 6 7 8 9 10] );
ull_mat = uint64( [10 20 30; 40 50 60; 70 80 90] );


if (read_data_from_cpp)
    
    %% Open and read file
    f_io = FileIO(filename, FileIO.in);

    f_io.printHeader();

    d_mat = f_io.read('d_mat');
    d = f_io.read('d');
    ull_mat = f_io.read('ull_mat');
    i = f_io.read('i');
    i_rowvec = f_io.read('i_rowvec');
    f = f_io.read('f');
    b = f_io.read('b');
    f_vec = f_io.read('f_vec');
    u = f_io.read('u');
    
    w = 10;
    fprintf('\n======================\n');
    b
    i
    u
    f
    d
    d_mat
    f_vec
    i_rowvec
    ull_mat
    
else
    %% Create and write file
    f_io = FileIO(filename, bitor(FileIO.out,FileIO.trunc) );
    f_io.write('b', b);
    f_io.write('i', i);
    f_io.write('u', u);
    f_io.write('f', f);
    f_io.write('d', d);
    f_io.write('d_mat', d_mat);
    f_io.write('f_vec', f_vec);
    f_io.write('i_rowvec', i_rowvec);
    f_io.write('ull_mat', ull_mat);

    f_io.printHeader();
    f_io.close();
    
end





