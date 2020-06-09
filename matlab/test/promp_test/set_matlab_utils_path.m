function set_matlab_utils_path()

path = strrep(mfilename('fullpath'), 'set_matlab_utils_path','');

addpath([path 'utils/']);
addpath([path 'MP_lib/']);
addpath([path '../../math_lib/']);

end
