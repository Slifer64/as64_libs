function set_matlab_utils_path()

path = strrep(mfilename('fullpath'), 'set_matlab_utils_path','');

addpath([path '../../io_lib/']);
import_io_lib();

end