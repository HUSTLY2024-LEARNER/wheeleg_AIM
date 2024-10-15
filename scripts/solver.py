import SOLVER_
from config import Config


class SolverConfig(Config):
    def __init__(self, json_file_path):
        super().__init__(json_file_path, 'pose_solver')

solver_config = SolverConfig('../scripts/config.json')

# m.def("bkg_solver_run", bkg_solver_run, py::arg("fx"), py::arg("fy"), py::arg("u0"), py::arg("v0"),
#                                         py::arg("k1"), py::arg("k2"), py::arg("p1"), py::arg("p2"),
#                                         py::arg("k3"), py::arg("cameraPitchAngle"), 
#                                         py::arg("camera_trans_x"), py::arg("camera_trans_y"), py::arg("camera_trans_z"));
# }

# SOLVER_.bkg_solver_run(1294.09084241273,1290.02123398661,640.545501071714,551.669780026064,-0.208520312255198,0.164286044558854,0,0,0,0,0, -0.106, 0.005)

SOLVER_.bkg_solver_run(
    solver_config.fx,
    solver_config.fy,
    solver_config.u0,
    solver_config.v0,
    solver_config.k1,
    solver_config.k2,
    solver_config.p1,
    solver_config.p2,
    solver_config.k3,
    solver_config.cameraPitchAngle,
    solver_config.cameraYawAngle,
    solver_config.camera_trans_x,
    solver_config.camera_trans_y,
    solver_config.camera_trans_v
    )
