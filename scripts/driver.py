import DRIVER_
from config import Config


class DriverConfig(Config):
    def __init__(self, json_file_path):
        super().__init__(json_file_path, 'driver')

driver_config = DriverConfig('../scripts/config.json')

# PYBIND11_EMBEDDED_MODULE(DRIVER_, m) {
#     namespace py = pybind11;
#     m.def("bkg_driver_auto_restart", bkg_driver_auto_restart, py::arg("device_path"), py::arg("baud_rate"), py::arg("camera_sn"), py::arg("exposure_time"));
# }

# DRIVER_.bkg_driver_auto_restart("/dev/ttyACM0", 115200, "KE0200060398", 5000)

DRIVER_.bkg_driver_auto_restart(
    driver_config.device_path, 
    driver_config.baud_rate, 
    driver_config.camera_sn, 
    driver_config.exposure_time
    )