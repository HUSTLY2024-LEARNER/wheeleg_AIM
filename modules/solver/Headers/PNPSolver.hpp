#pragma once

#include <vector>
#include <BBoxes.h>
#include "solver.hpp"

using namespace DETECTOR;
using namespace cv;
using namespace std;
namespace SOLVER 
{
    class PNPSolverBase
    {
    public:
        PNPSolverBase() = default;
        ~PNPSolverBase() = default;

        cv::Mat image_reproject;

        // 在子类中override，返回的是armor_to_world矩阵
        virtual IndexedArmorPoses solveArmorPoses(const std::vector<std::pair<int,BBox>> &bboxes_with_index, DRIVER::SerialReadData::IMU_Flag imu_flag, ARMOR_SIZE armor_size)=0;

        void setCamera(double fx, double fy, double u0, double v0, double k1, double k2, double p1, double p2, double k3, double cameraPitchAngle, double cameraYawAngle, Eigen::Vector3d cameraTrans) {
            // 相机内参
            cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
            cameraMatrix.ptr<double>(0)[0] = fx;
            cameraMatrix.ptr<double>(0)[2] = u0;
            cameraMatrix.ptr<double>(1)[1] = fy;
            cameraMatrix.ptr<double>(1)[2] = v0;
            cameraMatrix.ptr<double>(2)[2] = 1.0f;
            // 畸变系数
            distortionCoefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
            distortionCoefficients.ptr<double>(0)[0] = k1;
            distortionCoefficients.ptr<double>(1)[0] = k2;
            distortionCoefficients.ptr<double>(2)[0] = p1;
            distortionCoefficients.ptr<double>(3)[0] = p2;
            distortionCoefficients.ptr<double>(4)[0] = k3;

            // 相机相对于枪口的俯仰角：度。（英雄可能会向下看方便吊射，一般为负值）
            this->cameraPitchAngle = cameraPitchAngle;
            this->cameraYawAngle = cameraYawAngle;
            // 相机相对于枪口的平移向量：
            this->cameraTrans = cameraTrans;

            // 初始化相机到云台的变换矩阵
            updateCameraToGimbal();
        }

        //////////////////////////////////////////////////
        // Gimbal与World的位姿信息与IMU有关
        void updateGimbalToWorld(DRIVER::SerialReadData::IMU_Flag imu_flag) {
            /*
                imu_flag 为串口位姿信息
                直接将相机角度与云台角度相加
            */
            float degree2rad = 3.1415926 / 180;
            float yaw = (imu_flag.yaw_now + this->cameraYawAngle) * degree2rad;
            float pitch = (imu_flag.pitch_now + this->cameraPitchAngle) * degree2rad; 
            float roll = 0; // 通常情况，TODO：若串口发送roll此处应相应修改
            // float roll = imu_flag.roll_now; TODO
            Eigen::Matrix3d gimbal_rotation_matrix;
            // 旋转顺序按yaw-pitch-roll
            gimbal_rotation_matrix =  Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitY()) * // yaw
                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) * // pitch
                                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()); // roll
            
            gimbal_to_world = Sophus::SE3<double>(gimbal_rotation_matrix, Eigen::Vector3d(0, 0, 0)); // x,y,z平移均为0，世界坐标系原点与云台原点重合
        }
        // Camera与Gimbal的位姿信息与相机的云台的相对关系有关
        void updateCameraToGimbal() {
            float degree2rad = 3.1415926 / 180;
            Eigen::Vector3d euler_angle(0, 0, 0); // 由于是pitch角度的旋转，所以只有pitch对应的x轴有值
            Eigen::Matrix3d rotation_matrix;

            rotation_matrix = Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ()) * // yaw
                            Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX()) * // pitch
                            Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()); // roll

            camera_to_gimbal = Sophus::SE3<double>(rotation_matrix, cameraTrans);
        }
        /////////////////////////////////////////////////

    protected:
        //////////////////////////////////////////////////
        // 相机参数：通过标定获取
        cv::Mat cameraMatrix;
        cv::Mat distortionCoefficients;

        // 相机位姿：以下两个参数需要从机械图纸获取
        double cameraPitchAngle = 0; // 相机相对于枪口的俯仰角：度。（英雄可能会向下看方便吊射，一般为负值）
        double cameraYawAngle = 0;
        Eigen::Vector3d cameraTrans; // 相机相对于枪口的平移向量：相机光心到枪口的向量
        //////////////////////////////////////////////////

        //////////////////////////////////////////////////
        // 以下为变换矩阵，通过右乘齐次坐标可以进行相应的变换
        // 只保留解算过程中每一个过程的矩阵。需要运用连续变换时，直接相乘；需要逆变换，直接求逆。此处不要再设置新的变换矩阵。
        /*
            相机坐标系
            原点：相机光心
            x: 指向相机右侧
            y: 指向相机上方
            z：指向相机前方
        */ 
        cv::Mat rvec; // 旋转向量 OpenCV
        cv::Mat tvec; // 平移向量 OpenCV
        
        Sophus::SE3<double> armor_to_camera;  
        Sophus::SE3<double> camera_to_gimbal;
        Sophus::SE3<double> gimbal_to_world;
        //////////////////////////////////////////////////

        //////////////////////////////////////////////////
        // 装甲板的物理信息，用于PNP解算
        float length_of_small = 0.0675f;        
        float height_of_small = 0.0275f; 
        float length_of_big = 0.1125f;          
        float height_of_big = 0.0275f; 
        
        // 小装甲板3d坐标
        vector<Point3f> points_small_3d = {Point3f(-length_of_small, -height_of_small, 0.f),
                                                Point3f(length_of_small, -height_of_small, 0.f),
                                                Point3f(length_of_small, height_of_small, 0.f),
                                                Point3f(-length_of_small, height_of_small, 0.f)};
        
        //  大装甲板3d坐标
        vector<Point3f> points_large_3d = {Point3f(-length_of_big, -height_of_big, 0.f),
                                                Point3f(length_of_big, -height_of_big, 0.f),
                                                Point3f(length_of_big, height_of_big, 0.f),
                                                Point3f(-length_of_big, height_of_big, 0.f)};
        //////////////////////////////////////////////////
    };

    class NormalPNPSolver : public PNPSolverBase {
    public:
        IndexedArmorPoses solveArmorPoses(const std::vector<std::pair<int,BBox> > &bboxes_with_index, DRIVER::SerialReadData::IMU_Flag imu_flag, ARMOR_SIZE armor_size) override;        
    };

    class AccuratePNPSolver : public PNPSolverBase {
    public:
        AccuratePNPSolver();
        IndexedArmorPoses solveArmorPoses(const std::vector<std::pair<int,BBox> > &bboxes_with_index, DRIVER::SerialReadData::IMU_Flag imu_flag, ARMOR_SIZE armor_size) override;   
    private:
        // RPY 顺序，对应 eulerAngles(2, 1, 0)
        Eigen::AngleAxisd prior_roll_rotate;
        Eigen::AngleAxisd prior_pitch_rotate;
        Eigen::AngleAxisd prior_pitch_and_roll_rotate;
        Eigen::Matrix3d iterated_rotation_matrix;
        double pnp_yaw_deviation;
        double reprojection_cost;
        double normal_pnp_yaw;
        double getProjectionCost(const std::vector<cv::Point2f>& detected_points, const std::vector<cv::Point2f>& projected_points);
        double trichotomyFitYaw(const std::vector<cv::Point2f>& detected_points, ARMOR_SIZE armor_size);
    };

    class PNPSolverFactory {
    public:
        static std::unique_ptr<PNPSolverBase> createPNPSolver(const std::string &type) {
            if(type=="normal")
                return std::make_unique<NormalPNPSolver>();
            else if(type=="accurate")
                return std::make_unique<AccuratePNPSolver>();
            else 
                return std::make_unique<NormalPNPSolver>();
            }
    };
} // namespace SOLVER