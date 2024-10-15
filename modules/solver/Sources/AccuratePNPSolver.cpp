#include "PNPSolver.hpp"
#include "umt.hpp"

namespace SOLVER 
{
#define DETECTOR_ERROR_PIXEL_BY_SLOPE 100.0

    AccuratePNPSolver::AccuratePNPSolver()
    {
        this->prior_roll_rotate = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
        this->prior_pitch_rotate = Eigen::AngleAxisd((-15. * M_PI) / 180., Eigen::Vector3d::UnitX());
        this->prior_pitch_and_roll_rotate = this->prior_pitch_rotate * this->prior_roll_rotate;
        this->pnp_yaw_deviation = 30. * M_PI /180.;
    };

    // 重写PNPSolverBase的solveArmorPoses函数
    IndexedArmorPoses AccuratePNPSolver::solveArmorPoses(const std::vector<std::pair<int,BBox> > &bboxes_with_index, DRIVER::SerialReadData::IMU_Flag imu_flag, ARMOR_SIZE armor_size) 
    {
        umt::Publisher<std::pair<std::pair<cv::Mat,cv::Mat>, Sophus::SE3<double>>> solve_matrix_pub("solve_matrix");

        IndexedArmorPoses armor_poses;
        for (auto bbox_with_index : bboxes_with_index) {
            auto bbox = bbox_with_index.second;
            int index = bbox_with_index.first;
            // 从bbox中获取装甲板的3d坐标
            vector<Point2f> corners;
            for (int i = 0; i < 4; i++) {
                corners.push_back(bbox.corners[i]);
            }

            cv::Mat m_R; // 旋转矩阵 OpenCV 矩阵
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

            double temp = rvec.ptr<double>(0)[1];
            rvec.ptr<double>(0)[1] = rvec.ptr<double>(0)[2];
            rvec.ptr<double>(0)[2] = temp;

            cv::Rodrigues(rvec, m_R); // 旋转向量转换为旋转矩阵
            cv::cv2eigen(m_R, e_R);
            cv::cv2eigen(tvec,e_T);

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

            // COUT("YAW :"<<yaw, CYAN);

            // 三分法迭代求解yaw
            this->normal_pnp_yaw = yaw;
            // double accurate_yaw = trichotomyFitYaw(corners, armor_size);

            armor_to_camera = Sophus::SE3<double>(e_R, e_T);
            updateGimbalToWorld(imu_flag);
            auto armor_to_world = gimbal_to_world * camera_to_gimbal * armor_to_camera;
            ArmorPose pose;
            pose.translate = armor_to_world.translation();
            pose.yaw_world = yaw - imu_flag.yaw_now * M_PI / 180.0;

            armor_poses.push_back(std::make_pair(index, pose));
        }
        auto solve_matrix = umt::ObjManager<std::pair<std::pair<cv::Mat,cv::Mat>, Sophus::SE3<double>>>::find_or_create("solve_matrix");
        *solve_matrix.get() = std::make_pair(std::make_pair(cameraMatrix,distortionCoefficients),gimbal_to_world * camera_to_gimbal);
        
        return armor_poses;
    }

    double AccuratePNPSolver::trichotomyFitYaw(const std::vector<cv::Point2f>& detected_points, ARMOR_SIZE armor_size)
    {
        // 三分法，从pnp解出的yaw上下加上pnp_yaw_deviation作为两端
        double left_yaw = -M_PI_4; // this->normal_pnp_yaw - pnp_yaw_deviation;
        double right_yaw = M_PI_4; //this->normal_pnp_yaw + pnp_yaw_deviation;
        double mid_left_yaw = 0.0, mid_right_yaw = 0.0;
        double epsilon = 1e-2, cost_left = 0.0, cost_right = 0.0;

        std::vector<cv::Point2f> projected_points_left, projected_points_right;

        do{
            mid_left_yaw = (2 * left_yaw + right_yaw) / 3;
            mid_right_yaw = (left_yaw + 2 * right_yaw) / 3;

            cv::Mat rvec_left, cv_rotation_matrix_left;
            Eigen::Matrix3d eigen_rotation_matrix_left = (Eigen::AngleAxisd(mid_left_yaw, Eigen::Vector3d::UnitY()) * this->prior_pitch_and_roll_rotate).toRotationMatrix();
            cv::eigen2cv(eigen_rotation_matrix_left, cv_rotation_matrix_left);
            cv::Rodrigues(cv_rotation_matrix_left, rvec_left);

            cv::Mat rvec_right, cv_rotation_matrix_right;
            Eigen::Matrix3d eigen_rotation_matrix_right = (Eigen::AngleAxisd(mid_right_yaw, Eigen::Vector3d::UnitY()) * this->prior_pitch_and_roll_rotate).toRotationMatrix();
            cv::eigen2cv(eigen_rotation_matrix_right, cv_rotation_matrix_right);
            cv::Rodrigues(cv_rotation_matrix_right, rvec_right);

            if(armor_size == ARMOR_SIZE::BIG_ARMOR) {
                cv::projectPoints(points_large_3d, rvec_left, tvec, cameraMatrix, distortionCoefficients, projected_points_left);
                cv::projectPoints(points_large_3d, rvec_right, tvec, cameraMatrix, distortionCoefficients, projected_points_right);
            } else {
                cv::projectPoints(points_small_3d, rvec_left, tvec, cameraMatrix, distortionCoefficients, projected_points_left);
                cv::projectPoints(points_small_3d, rvec_right, tvec, cameraMatrix, distortionCoefficients, projected_points_right);
            }

            cost_left = getProjectionCost(detected_points, projected_points_left);
            cost_right = getProjectionCost(detected_points, projected_points_right);

            if (cost_left < cost_right) {
                right_yaw = mid_right_yaw;
            } else {
                left_yaw = mid_left_yaw;
            }
        } while (right_yaw - left_yaw > epsilon); 

        // 重投影可视化
        image_reproject = cv::Mat::zeros(600, 600, CV_8UC3);
        int i = 0;
        for (const auto& point : detected_points) {
            cv::circle(image_reproject, point, 2, cv::Scalar(0, 255, 0), -1); // 在图像上绘制圆
            cv::putText(image_reproject, std::to_string(i), point, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
            i++;
        }
        for (const auto& point : projected_points_left) {
            cv::circle(image_reproject, point, 2, cv::Scalar(255, 0, 0), -1); // 在图像上绘制圆
        }
        for (const auto& point : projected_points_right) {
            cv::circle(image_reproject, point, 2, cv::Scalar(0, 0, 255), -1); // 在图像上绘制圆
        }

        return (left_yaw + right_yaw) / 2.0;
    }

    inline double get_abs_angle(const Eigen::Vector2d& vec_a, const Eigen::Vector2d& vec_b) {
        return std::acos(vec_a.dot(vec_b) / (vec_a.norm() * vec_b.norm()));
    }

    inline double square(double x) {
        return x * x;
    }

    double AccuratePNPSolver::getProjectionCost(const std::vector<cv::Point2f>& detected_points, const std::vector<cv::Point2f>& projected_points)
    {
        std::size_t size = detected_points.size();
        std::vector<Eigen::Vector2d> detected_points_eigen;
        std::vector<Eigen::Vector2d> projected_points_eigen;
        for(std::size_t i =0u; i < size; ++i)
        {
            detected_points_eigen.emplace_back(detected_points[i].x, detected_points[i].y);
            projected_points_eigen.emplace_back(projected_points[i].x, projected_points[i].y);            
        }
        double cost = 0.;
        for (std::size_t i = 0u; i < size; ++i) {
            std::size_t p = (i + 1u) % size;
            // i - p 构成线段。过程：先移动起点，再补长度，再旋转
            Eigen::Vector2d detected_d = detected_points_eigen[p] - detected_points_eigen[i]; 
            Eigen::Vector2d projected_d = projected_points_eigen[p] - projected_points_eigen[i];
            // pixel_distance：平移伸缩误差，长度差代价 + 起点差代价(1 / 2)，yaw达到0度左右应该抛弃该项
            double pixel_distance = (
                0.5 * ((detected_points_eigen[i] - projected_points_eigen[i]).norm() + (detected_points_eigen[p] - projected_points_eigen[p]).norm())
                + std::fabs(detected_d.norm() - projected_d.norm())
                ) / detected_d.norm();

            double angular_dis = detected_d.norm() * get_abs_angle(detected_d, projected_d) / detected_d.norm();
            // 弧度差代价（0 度左右占比应该大）
            double cost_i = square(pixel_distance * std::sin(this->normal_pnp_yaw))
                + square(angular_dis * std::cos(this->normal_pnp_yaw)) * DETECTOR_ERROR_PIXEL_BY_SLOPE;
            cost += std::sqrt(cost_i);
        }
        return cost;
    }
}