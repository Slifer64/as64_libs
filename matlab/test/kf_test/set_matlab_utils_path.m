function set_matlab_utils_path()

path = strrep(mfilename('fullpath'), 'set_matlab_utils_path','');

addpath([path 'utils/']);
addpath([path 'utils/quaternions/']);
addpath([path 'utils/system_model/']);
addpath([path '../../kf_lib/']);

import_kf_lib();

end