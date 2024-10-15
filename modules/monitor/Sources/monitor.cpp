#include <thread>
#include <chrono>

#include <driver.hpp>
#include <SmartLog.hpp>
#include <Eigen/Dense>

#include <pybind11/numpy.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <umt.hpp>


#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

#include <monitor.hpp>

namespace py = pybind11;

py::array_t<uint8_t> cvMat2npArray(const cv::Mat &mat)
{
    py::array_t<uint8_t> array({mat.rows, mat.cols, mat.channels()});
    cv::Mat ref_mat(mat.rows, mat.cols, CV_8UC(mat.channels()), array.mutable_data());
    mat.copyTo(ref_mat);
    return array;
}

UMT_EXPORT_MESSAGE_ALIAS(cvMat, cv::Mat, c)
{
    c.def(py::init<cv::Mat>());
    c.def("get_nparray", cvMat2npArray);
}

using namespace LY_UTILS;

namespace MONITOR
{
    std::vector<cv::Point2f> projectWorldToCamera(Eigen::Vector3d trans, Sophus::SE3<double> camera_to_world, cv::Mat camera_matrix, cv::Mat distortionCoefficients)
    {   
        trans[2] = -trans[2];
        std::swap(trans[1], trans[2]);
         std::vector<cv::Point2f> armor;
        Eigen::Matrix3d rotate_matrix = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()).matrix() * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()).matrix() * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()).matrix();
        Sophus::SO3<double> rotate(rotate_matrix);
        Sophus::SE3<double> armor_in_world = Sophus::SE3<double>(rotate.inverse(), trans);
        Sophus::SE3<double> armor_in_camera = camera_to_world.inverse() * armor_in_world;
         cv::Mat R;
        cv::eigen2cv(armor_in_camera.so3().matrix(), R);
        cv::Mat rv;
        cv::Rodrigues(R, rv);
        // std::cout<<"translate:"<<armor_in_camera.translation()<<std::endl;
        cv::Mat tv = (cv::Mat_<double>(3,1) <<armor_in_camera.translation()[0], armor_in_camera.translation()[1], armor_in_camera.translation()[2]);

        double temp = rv.ptr<double>(0)[1];
        rv.ptr<double>(0)[1] = rv.ptr<double>(0)[2];
        rv.ptr<double>(0)[2] = temp;
        
         std::vector<cv::Point3f> points_;
        points_.push_back(cv::Point3f(0.f,0.f,0.f));
        points_.push_back(cv::Point3f(-0.05f,0.f,0.f));
        points_.push_back(cv::Point3f(0.f,-0.05f,0.f));
        points_.push_back(cv::Point3f(0.f,0.f,-0.05f));
         cv::projectPoints(points_, rv, tv, camera_matrix, distortionCoefficients, armor);
 
        return armor;
    }

    void Recorder::write(const cv::Mat& frame) {
        try
        {
            if (!this->is_writer_start)
            {
                this->writer.open("../autoaim" + std::to_string(clock()) + ".avi", this->mode, 40.0, cv::Size(1280, 1024), true);
                if (!this->writer.isOpened())
                {
                    std::cerr << "ERROR WHEN OPEN WRITE MOVIE" << std::endl;
                }
                else
                {
                    this->is_writer_start = true;
                    this->writer_start_time = std::chrono::steady_clock::now();
                }
            }
            if (this->writer.isOpened())
            {
                auto start_write_time = std::chrono::steady_clock::now();
                this->writer.write(frame);
                auto end_write_time = std::chrono::steady_clock::now();
            }
            std::chrono::steady_clock::time_point writer_this_time = std::chrono::steady_clock::now();
            // 两分钟之后保存视频
            if (std::chrono::duration_cast<std::chrono::seconds>(writer_this_time - this->writer_start_time).count() > 120)
            {
                // LOG(WARNING) << "SAVE VIDEO";
                this->writer.release(); // 保存视频
                this->is_writer_start = false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void monitor_run()
    {


        umt::Subscriber<DRIVER::PicStamp> pic_sub("image_used");
        Recorder recorder;

        while (true)
        {
            try
            {
                // const auto &gates = track_sub.pop();
                //const auto &points = points_sub.pop();

                // const auto &point = point_hit_sub.pop();
                // auto *this_solve_matrix = solve_matrix.get();
                const auto &img = pic_sub.pop();

                recorder.write(img.pic);

                // const auto &point = buff_point_sub.pop();
                COUT("SHOW", CYAN);
  
            }
            catch (umt::MessageError &e)
            {
                // COUT("[WARNING] 'image_raw' " << e.what(), RED);
            }
        }
    }

    void bkg_monitor_run()
    {
        std::thread([]()
                    { monitor_run(); })
            .detach();
    }

    PYBIND11_EMBEDDED_MODULE(MONITOR_, m)
    {
        namespace py = pybind11;
        m.def("bkg_monitor_run", bkg_monitor_run);
    }
} // namespace MONITOR
