#include "PNPSolver.hpp"

namespace SOLVER 
{
    // 重写PNPSolverBase的solveArmorPoses函数
    IndexedArmorPoses NormalPNPSolver::solveArmorPoses(const std::vector<std::pair<int,BBox> > &bboxes_with_index, DRIVER::SerialReadData::IMU_Flag imu_flag, ARMOR_SIZE armor_size) {
        IndexedArmorPoses armor_poses;
        for (auto bbox_with_index : bboxes_with_index) {
            auto bbox = bbox_with_index.second;
            auto index = bbox_with_index.first;

            // 从bbox中获取装甲板的3d坐标
            vector<Point2f> corners;
            for (int i = 0; i < 4; i++) {
                corners.push_back(bbox.corners[i]);
            }
            
            cv::Mat m_R; // 旋转矩阵 OpenCV 矩阵
            cv::Mat m_T; // 平移向量 OpenCV 矩阵
            Eigen::Matrix3d e_R; // 旋转矩阵 Eigen
            Eigen::Vector3d e_T; // 平移向量 Eigen
           
            if (armor_size == ARMOR_SIZE::BIG_ARMOR) {
                // large armor
                solvePnP(points_large_3d, corners, cameraMatrix, distortionCoefficients,
                        rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            }
            else {
                // small armor
                solvePnP(points_small_3d, corners, cameraMatrix, distortionCoefficients,
                        rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            }
            
            // 输出距离
            float distance = sqrt(m_T.at<double>(0, 0) * m_T.at<double>(0, 0) +
                                m_T.at<double>(1, 0) * m_T.at<double>(1, 0) +
                                m_T.at<double>(2, 0) * m_T.at<double>(2, 0));

            double temp = rvec.ptr<double>(0)[1];
            rvec.ptr<double>(0)[1] = rvec.ptr<double>(0)[2];
            rvec.ptr<double>(0)[2] = temp;

            cv::Rodrigues(rvec, m_R); // 旋转向量转换为旋转矩阵

            Sophus::SO3<double> armor_rotate(e_R);
            double yaw = armor_rotate.matrix().eulerAngles(2,1,0)[0];
            double angle_1 = armor_rotate.matrix().eulerAngles(2,1,0)[1];
            double angle_2 = armor_rotate.matrix().eulerAngles(2,1,0)[2];

            if(yaw > M_PI_2)
            {
                if(angle_1 > M_PI_4 && angle_2 < -M_PI_4)
                {
                    yaw = M_PI - yaw;
                }
                else if (angle_1 > M_PI_4 && angle_2 > M_PI_4)
                {
                    yaw = M_PI - yaw;
                }
                else if(angle_1 < -M_PI_4 && angle_2 > M_PI_4)
                {
                    yaw = yaw - M_PI;
                }
                else if(angle_1 < -M_PI_4 && angle_2 < -M_PI_4)
                {
                    yaw = yaw - M_PI;
                }
            }

            // 从OpenCV的矩阵转换为Eigen的矩阵
            cv::cv2eigen(m_R, e_R);
            cv::cv2eigen(m_T, e_T); 

            // 更新装甲板到相机的变换矩阵
            armor_to_camera = Sophus::SE3<double>(e_R, e_T);
            updateGimbalToWorld(imu_flag);
            auto armor_to_world = gimbal_to_world * camera_to_gimbal * armor_to_camera;
            ArmorPose pose;
            pose.translate = armor_to_world.translation();
            pose.yaw_world = yaw + imu_flag.yaw_now * M_PI / 180.0;
            armor_poses.push_back(std::make_pair(index, pose));
        }
        return armor_poses;
    }
} // namespace SOLVER