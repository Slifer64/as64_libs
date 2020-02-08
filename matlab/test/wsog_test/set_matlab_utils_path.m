function set_matlab_utils_path()

path = strrep(mfilename('fullpath'), 'set_matlab_utils_path','');

addpath([path 'utils/']);
addpath([path '../../dmp_lib/']);
addpath([path '../../io_lib/']);
addpath([path '../../plot_lib/']);

import_dmp_lib();
import_io_lib();
import_plot_lib();

end
