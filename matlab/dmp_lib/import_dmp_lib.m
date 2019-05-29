
dmp_lib_path = mfilename('fullpath');
dmp_lib_path = strrep(dmp_lib_path, 'import_dmp_lib','');

addpath(dmp_lib_path);
addpath([dmp_lib_path '/DMP/']);
addpath([dmp_lib_path '/CanonicalClock/']);
addpath([dmp_lib_path '/GatingFunction/']);
addpath([dmp_lib_path '/trainMethods/']);
addpath([dmp_lib_path '/enums/']);
addpath([dmp_lib_path '/math/']);
addpath([dmp_lib_path '/ProMP/']);