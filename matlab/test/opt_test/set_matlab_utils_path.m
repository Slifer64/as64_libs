function set_matlab_utils_path()

path = strrep(mfilename('fullpath'), 'set_matlab_utils_path','');

addpath([path 'Objectives/']);
addpath([path 'opt_lib/']);

import_opt_lib();

end
