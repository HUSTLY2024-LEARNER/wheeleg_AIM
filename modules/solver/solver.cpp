#include <thread>

#include <umt/Message.hpp>
#include <umt/ObjManager.hpp>
#include <solver.hpp>
#include <tracker.hpp>
#include <PNPSolver.hpp>
#include <SmartLog.hpp>

using namespace LY_UTILS;
#define THIS_ARMOR_SIZE(x) (x->at(static_cast<ENEMY_TYPE>(track_pack.bboxes_with_index.at(0).second.tag_id)))
#define THIS_ARMOR_TYPE(x) (static_cast<ENEMY_TYPE>(x.bboxes_with_index.at(0).second.tag_id))

namespace SOLVER
{
    void solver_run(double fx, double fy, double u0, double v0, 
                    double k1, double k2, double p1, double p2, double k3, 
                    double cameraPitchAngle, double cameraYawAngle,
                    double camera_trans_x, double camera_trans_y, double camera_trans_z)
    { 
        umt::Subscriber<TRACKER::TrackingPackage> track_sub("tracking_pack");
        umt::Publisher<SolutionPackage> solution_pub("solution_pack");
        umt::Publisher<cv::Mat> image_pub("project_points");

        // "normal" or "accurate"
        std::unique_ptr<PNPSolverBase> pnp_solver = PNPSolverFactory::createPNPSolver("accurate");

        pnp_solver->setCamera(
            fx, fy, u0, v0, // fx, fy, cx, cy
            k1, k1, p1, p2, k3, // k1, k2, p1, p2, k3
            cameraPitchAngle, cameraYawAngle, // cameraPitchAngle
            Eigen::Vector3d(camera_trans_x, camera_trans_y, camera_trans_z)); // cameraTrans

        auto armor_size_map = umt::ObjManager<ARMOR_SIZE_MAP>::find_or_create("armor_size_map");

        while(true)
        { 
            try{
                auto track_pack = track_sub.pop_for(50);    

                auto *this_frame_armor_size_map = armor_size_map.get();
                ARMOR_SIZE armor_size = THIS_ARMOR_SIZE(this_frame_armor_size_map);
                if(armor_size != ARMOR_SIZE::UNKNOW)
                {
                    IndexedArmorPoses armor_poses = pnp_solver->solveArmorPoses(track_pack.bboxes_with_index, track_pack.imu_flag,armor_size);

                    image_pub.push(pnp_solver->image_reproject);
                    solution_pub.push({armor_poses,THIS_ARMOR_TYPE(track_pack),track_pack.move_status,track_pack.imu_flag,track_pack.time_stamp});
                }
                
                } catch (umt::MessageError_Timeout &e) {
                    //规定时限内未获取识别结果
                    //COUT("[WARNING] 'track_pack' "<<e.what(),RED);
                } catch (umt::MessageError &e) {
                    COUT("[WARNING] 'track_pack' "<<e.what(),RED);
                }
        }
    }
    
    void bkg_solver_run(double fx, double fy, double u0, double v0, 
                        double k1, double k2, double p1, double p2, double k3, 
                        double cameraPitchAngle, double cameraYawAngle,
                        double camera_trans_x, double camera_trans_y, double camera_trans_z)
    {
        std::thread([=](){
            solver_run(fx, fy, u0, v0, k1, k2, p1, p2, k3, cameraPitchAngle, cameraYawAngle, camera_trans_x, camera_trans_y, camera_trans_z);
        }).detach();
    }

PYBIND11_EMBEDDED_MODULE(SOLVER_, m) {
namespace py = pybind11;
m.def("bkg_solver_run", bkg_solver_run, py::arg("fx"), py::arg("fy"), py::arg("u0"), py::arg("v0"),
                                        py::arg("k1"), py::arg("k2"), py::arg("p1"), py::arg("p2"),
                                        py::arg("k3"), py::arg("cameraPitchAngle"), py::arg("cameraYawAngle"), 
                                        py::arg("camera_trans_x"), py::arg("camera_trans_y"), py::arg("camera_trans_z"));
}
}