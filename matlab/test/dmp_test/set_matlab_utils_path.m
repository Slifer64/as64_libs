function set_matlab_utils_path()

addpath('utils/');
addpath('utils/lib/dmp_lib/');
addpath('utils/lib/io_lib/');
addpath('utils/lib/plot_lib/');

import_dmp_lib();
import_io_lib();
import_plot_lib();

end
