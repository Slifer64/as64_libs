function set_matlab_utils_path()

path = strrep(mfilename('fullpath'), 'set_matlab_utils_path','');

addpath([path 'utils/']);
addpath([path '../../math_lib/']);
% addpath([path '../../kf_lib/']);

import_math_lib();
% import_kf_lib();

end