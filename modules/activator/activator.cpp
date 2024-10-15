#include <activator.hpp>
#include <driver.hpp>
#include <detector.hpp>
#include <BuffPredictor.hpp>
#include <AngleTimeSolver.hpp>
#include <thread>
#include <umt.hpp>
#include <utils.h>
#include <predictor.hpp>

using namespace LY_UTILS;

namespace ACTIVATOR
{
#define LARGE_BUFF_MODE(x) ((int)x.mode_want==static_cast<int>(DRIVER::AimMode::AIM_LARGE_BUFF))

    void onePointCalc(float rcv_yaw, float rcv_pitch, float &yaw, float &pitch, const cv::Point2f &target)
    {
        const static double fx = 1283.217;
        const static double u0 = 624.2064;
        const static double fy = 1279.5787;
        const static double v0 = 485.6023;
        
        // 转弧度
        rcv_yaw = rcv_yaw * M_PI / 180.0f;
        rcv_pitch = rcv_pitch * M_PI / 180.0f;

        double x = (target.x - u0) / fx;
        double y = (target.y - v0) / fy;
        float delta_yaw = atan(x);
        float delta_pitch = atan(y);

        yaw = rcv_yaw - delta_yaw;
        pitch = rcv_pitch - delta_pitch;

        // 转角度
        yaw = yaw * 180.0f / M_PI;
        pitch = pitch * 180.0f / M_PI;

        // std::cout<<"_yaw: "<<yaw<<std::endl;
        // std::cout<<"_pitch: "<<pitch<<std::endl;
    }

    void activator_run()
    {
        umt::Subscriber<DETECTOR::BuffDetectionPackage> buff_detection_sub("buff_detection_pack");
        umt::Publisher<PredictionPackage> prediction_pub("prediction_pack");

        std::unique_ptr<BuffPredictor> buff_predictor = std::make_unique<BuffPredictor>();
        std::unique_ptr<AngleTimeSolver> angle_time_solver = std::make_unique<AngleTimeSolver>();

        umt::Publisher<cv::Point2f> buff_point_pub("buff_point");

        while(true){
            try {
                const auto& detection_pack = buff_detection_sub.pop_for(200);

                double update_time = buff_predictor->updateDeltaTime(detection_pack.time_stamp);
                buff_predictor->setT(update_time);

                // if(LARGE_BUFF_MODE(detection_pack.imu_flag))
                if(false)
                {
                    COUT("BIG", BLUE);
                    buff_predictor->setPredictMode(1); // 大符
                }
                else
                {
                    COUT("SMALL", BLUE);
                    buff_predictor->setPredictMode(0);
                }

                // buff_point_pub.push(detection_pack.buff_detection.pts[0]);

                std::vector<cv::Point2f> input_points;
                std::vector<cv::Point2f> output_points;

                input_points.emplace_back(detection_pack.buff_detection.pts[0]);
                input_points.emplace_back(detection_pack.buff_detection.pts[1]);
                cv::Mat cameraMatrix;
                cv::Mat distortionCoefficients;
    
                cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
                cameraMatrix.ptr<double>(0)[0] = 1283.217;
                cameraMatrix.ptr<double>(0)[2] = 624.2064;
                cameraMatrix.ptr<double>(1)[1] = 1279.5787;
                cameraMatrix.ptr<double>(1)[2] = 485.6032;
                cameraMatrix.ptr<double>(2)[2] = 1.0f;
                // 畸变系数
                distortionCoefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
                distortionCoefficients.ptr<double>(0)[0] = -0.202175;
                distortionCoefficients.ptr<double>(1)[0] = 0.13682;
                distortionCoefficients.ptr<double>(2)[0] = 0.;
                distortionCoefficients.ptr<double>(3)[0] = 0.;
                distortionCoefficients.ptr<double>(4)[0] = 0.;
                cv::undistortPoints(input_points, output_points, cameraMatrix, distortionCoefficients, cv::noArray(), cameraMatrix);

                float filte_angle = buff_predictor->OnePointPredict(detection_pack.buff_detection);

                Eigen::Vector3d actual_center = Eigen::Vector3d(0, 6.6, 1.05);
                Eigen::Vector3d shoot_likly_target = actual_center + cos(filte_angle) * Eigen::Vector3d::UnitX() * 0.7 + sin(filte_angle) * Eigen::Vector3d::UnitZ() * 0.7;
                float bullet_fly_time = angle_time_solver->getBulletFlyTime(shoot_likly_target);

                cv::Point2f target_2d_point = buff_predictor->OnePointPositonCalc(output_points.at(1), output_points.at(0), bullet_fly_time + 0.4, filte_angle);

                if (fabs(target_2d_point.x + 1) < 1e-3) // 大符模式下没拟合好
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(0));
                    continue;
                }

                buff_point_pub.push(target_2d_point);

                float yaw, pitch;
                onePointCalc(detection_pack.imu_flag.yaw_now, detection_pack.imu_flag.pitch_now, yaw, pitch, target_2d_point);

                prediction_pub.push({static_cast<uint8_t>(1), uint8_t(1), pitch + 4.5f, yaw + 0.5f});
            } catch (umt::MessageError &e) {
                COUT("[WARNING] 'buff_detection_pack' "<<e.what(),RED);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
    }

    void bkg_activator_run(){
    std::thread([=](){
        activator_run();
    }).detach();
    }

namespace py = pybind11;
PYBIND11_EMBEDDED_MODULE(ACTIVATOR_, m) {
    m.def("bkg_activator_run", bkg_activator_run);
}
}
