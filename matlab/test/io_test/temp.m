clc
close all
clear;

% s = warning('error', 'MATLAB:DELETE:Permission');
% warning('error', 'MATLAB:DELETE:FileNotFound');

delete temp.txt

try
    delete temp.txt
    disp('File deleted!');
catch
    disp('File does not exits...');
end

% warning(s);