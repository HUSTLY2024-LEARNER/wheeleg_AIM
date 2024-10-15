import DETECTOR_
from config import Config


class DetectorConfig(Config):
    def __init__(self, json_file_path):
        super().__init__(json_file_path, 'detector')

detector_config = DetectorConfig('../scripts/config.json')

# PYBIND11_EMBEDDED_MODULE(DETECTOR_, m) {
#     m.def("bkg_detector_run", bkg_detector_run, py::arg("armor_model_file"), py::arg("armor_model_file"), py::arg("buff_model_file"));
# }

# DETECTOR_.bkg_detector_run("../utils/models/armor_yolo_x.xml","../utils/models/armor_yolo_x.xml","../utils/models/buff_yolo.xml")

DETECTOR_.bkg_detector_run(
    detector_config.armor_model_file,
    detector_config.armor_model_file,
    detector_config.buff_model_file
    )