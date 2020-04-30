clc;
close all;
clear;

set_matlab_utils_path();

filename = 'data.bin';
delete(filename);

%% Generate some data
b = int8( 1 );
i = int32( -5 );
u = uint32( 16 );
f = single( 5.2 );
d = double( 6.7 );
d_mat = double( 100*rand(4,4) );
f_vec = single( 100*rand(5,1) );
i_rowvec = int32( 100*rand(1,6) );
ull_mat = uint64( 100*rand(1,6) );


%% Create and write file
f_io = FileIO(filename);

fprintf('\n==== Write::Header =====\n');

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

%% Open and read file
f_io = FileIO(filename);

fprintf('\n==== Read::Header =====\n');
f_io.printHeader();

d_mat2 = f_io.read('d_mat');
d2 = f_io.read('d');
ull_mat2 = f_io.read('ull_mat');
i2 = f_io.read('i');
i_rowvec2 = f_io.read('i_rowvec');
f2 = f_io.read('f');
b2 = f_io.read('b');
f_vec2 = f_io.read('f_vec');
u2 = f_io.read('u');

%% test if correct
w = 25;
fprintf('\n==== Testing =====\n');
fprintf('b-b2 %s: %d\n', repmat(' ',1,w-length('b-b2')), norm(double(b-b2)));
fprintf('i-i2 %s: %d\n', repmat(' ',1,w-length('i-i2')), norm(double(i-i2)));
fprintf('u-u2 %s: %d\n', repmat(' ',1,w-length('u-u2')), norm(double(u-u2)));
fprintf('f-f2 %s: %d\n', repmat(' ',1,w-length('f-f2')), norm(double(f-f2)));
fprintf('d-d2 %s: %d\n', repmat(' ',1,w-length('d-d2')), norm(double(d-d2)));
fprintf('d_mat-d_mat2 %s: %d\n', repmat(' ',1,w-length('d_mat-d_mat2')), norm(double(d_mat-d_mat2)));
fprintf('f_vec-f_vec2 %s: %d\n', repmat(' ',1,w-length('f_vec-f_vec2')), norm(double(f_vec-f_vec2)));
fprintf('i_rowvec-i_rowvec2 %s: %d\n', repmat(' ',1,w-length('i_rowvec-i_rowvec2')), norm(double(i_rowvec-i_rowvec2)));
fprintf('ull_mat-ull_mat2 %s: %d\n', repmat(' ',1,w-length('ull_mat-ull_mat2')), norm(double(ull_mat-ull_mat2)));



