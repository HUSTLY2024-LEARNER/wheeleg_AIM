#include "CenterStateEstimator.hpp"
#include"umt.hpp"
#define NO_DOUBLE_MEASURE false
#define HAVE_DOUBLE_MEASURE true

namespace PREDICTOR
{
    CenterStateEstimator::CenterStateEstimator()
    {
        this->center_state_filter = std::make_unique<ExtendedKalman<double,8,3>>();

        this->clearance_filter = std::make_unique<ExtendedKalman<double,1,1>>();
    };

    CenterStateEstimator::~CenterStateEstimator()
    {

    };

    void CenterStateEstimator::setT(double t)
    {
        this->update_time = t;
    }

    double CenterStateEstimator::doublePoseToClearance(SOLVER::ArmorPose armor_pose_1, SOLVER::ArmorPose armor_pose_2)
    {
        double x_1 = armor_pose_1.translate[0];
        double y_1 = armor_pose_1.translate[2];
        double x_2 = armor_pose_2.translate[0];
        double y_2 = armor_pose_2.translate[2];

        return std::sqrt((x_1 - x_2) * (x_1 - x_2) + (y_1 - y_2) * (y_1 - y_2));
    }

    void CenterStateEstimator::inputMeasurement(SOLVER::IndexedArmorPoses indexed_poses, Eigen::Vector2d two_yaw)
    {
        // 当有双装甲板观测时，对装甲板间距进行滤波
        if(indexed_poses.size() == 2)
        {
            double distance = (indexed_poses.at(0).second.translate.norm() + indexed_poses.at(0).second.translate.norm()) / 2.0;

            double clearance = doublePoseToClearance(indexed_poses.at(0).second, indexed_poses.at(1).second);
            Eigen::Matrix<double,1,1> clearance_measure;
            clearance_measure << clearance;
            this->clearance_filter->predict(clearance_state_to_measure, clearance_measure);
            Eigen::Matrix<double,1,1> clearance_state = this->clearance_filter->update();
            this->clearance = clearance_state[0];

            if((indexed_poses.at(0).first - indexed_poses.at(1).first == 1) || (indexed_poses.at(0).first - indexed_poses.at(1).first == -3))
            {
                if(indexed_poses.at(1).first == 0 || indexed_poses.at(1).first == 2) // (1)对应r_0，(0)对应r_1
                {
                    double yaw_1, yaw_2;
                    if(indexed_poses.at(1).first == 0)
                    {
                        yaw_1 = two_yaw[0];
                        yaw_2 = two_yaw[1];
                    }
                    else
                    {
                        yaw_1 = two_yaw[0] + M_PI;
                        yaw_2 = two_yaw[1] + M_PI;
                    }

                    this->center_state_to_armor_measure.inputClearance(this->clearance);

                    // 先用(1)更新滤波器，修改状态转移函数和process_noise后再用(0)更新滤波器
                    this->center_state_to_armor_measure.inputYawAngle(yaw_1); 
                    setProcessNoise(false);  
                    setTransitionMatrix(false);
                    this->center_state_to_armor_measure.setMeasuredArmorIndex(0);
                    Eigen::Vector3d xyz_1 = poseToXYZ(indexed_poses.at(1).second.translate);
                    this->center_state_filter->predict(center_state_to_armor_measure, xyz_1);
                    this->filtered_center_state = this->center_state_filter->update();

                    this->center_state_to_armor_measure.inputYawAngle(yaw_2); 
                    setProcessNoise(true);  
                    setTransitionMatrix(true);
                    this->center_state_to_armor_measure.setMeasuredArmorIndex(1);
                    Eigen::Vector3d xyz_2 = poseToXYZ(indexed_poses.at(0).second.translate);
                    this->center_state_filter->predict(center_state_to_armor_measure, xyz_2);
                    this->filtered_center_state = this->center_state_filter->update();
                }
                else // (0)对应r_0，(1)对应r_1
                {
                    double yaw_1, yaw_2;
                    if(indexed_poses.at(0).first == 0)
                    {
                        yaw_1 = two_yaw[0];
                        yaw_2 = two_yaw[1] + M_PI;
                    }
                    else
                    {
                        yaw_1 = two_yaw[0] + M_PI;
                        yaw_2 = two_yaw[1];
                    }

                    this->center_state_to_armor_measure.inputClearance(this->clearance);

                    // 先用(1)更新滤波器，修改状态转移函数和process_noise后再用(0)更新滤波器
                    this->center_state_to_armor_measure.inputYawAngle(yaw_1); 
                    setProcessNoise(false);  
                    setTransitionMatrix(false);
                    this->center_state_to_armor_measure.setMeasuredArmorIndex(0);
                    Eigen::Vector3d xyz_1 = poseToXYZ(indexed_poses.at(0).second.translate);
                    this->center_state_filter->predict(center_state_to_armor_measure, xyz_1);
                    this->filtered_center_state = this->center_state_filter->update();

                    this->center_state_to_armor_measure.inputYawAngle(yaw_2); 
                    setProcessNoise(true);  
                    setTransitionMatrix(true);
                    this->center_state_to_armor_measure.setMeasuredArmorIndex(1);
                    Eigen::Vector3d xyz_2 = poseToXYZ(indexed_poses.at(1).second.translate);
                    this->center_state_filter->predict(center_state_to_armor_measure, xyz_2);
                    this->filtered_center_state = this->center_state_filter->update();
                }
            }
            else if((indexed_poses.at(0).first - indexed_poses.at(1).first == -1) || (indexed_poses.at(0).first - indexed_poses.at(1).first == 3))
            {
                if(indexed_poses.at(0).first == 0 || indexed_poses.at(0).first == 2) // (0)对应r_0，(1)对应r_1
                {
                    double yaw_1, yaw_2;
                    if(indexed_poses.at(0).first == 0)
                    {
                        yaw_1 = two_yaw[0];
                        yaw_2 = two_yaw[1];
                    }
                    else
                    {
                        yaw_1 = two_yaw[0] + M_PI;
                        yaw_2 = two_yaw[1] + M_PI;
                    }
                    
                    this->center_state_to_armor_measure.inputClearance(this->clearance);

                    // 先用(1)更新滤波器，修改状态转移函数和process_noise后再用(0)更新滤波器
                    this->center_state_to_armor_measure.inputYawAngle(yaw_1); 
                    setProcessNoise(false);  
                    setTransitionMatrix(false);
                    this->center_state_to_armor_measure.setMeasuredArmorIndex(0);
                    Eigen::Vector3d xyz_1 = poseToXYZ(indexed_poses.at(0).second.translate);
                    this->center_state_filter->predict(center_state_to_armor_measure, xyz_1);
                    this->filtered_center_state = this->center_state_filter->update();

                    this->center_state_to_armor_measure.inputYawAngle(yaw_2); 
                    setProcessNoise(true);  
                    setTransitionMatrix(true);
                    this->center_state_to_armor_measure.setMeasuredArmorIndex(1);
                    Eigen::Vector3d xyz_2 = poseToXYZ(indexed_poses.at(1).second.translate);
                    this->center_state_filter->predict(center_state_to_armor_measure, xyz_2);
                    this->filtered_center_state = this->center_state_filter->update();
                }
                else // (1)对应r_0，(0)对应r_1
                {
                    double yaw_1, yaw_2;
                    if(indexed_poses.at(1).first == 0)
                    {
                        yaw_1 = two_yaw[0];
                        yaw_2 = two_yaw[1] + M_PI;
                    }
                    else
                    {
                        yaw_1 = two_yaw[0] + M_PI;
                        yaw_2 = two_yaw[1];
                    }

                    this->center_state_to_armor_measure.inputClearance(this->clearance);

                    // 先用(1)更新滤波器，修改状态转移函数和process_noise后再用(0)更新滤波器
                    this->center_state_to_armor_measure.inputYawAngle(yaw_1); 
                    setProcessNoise(false);  
                    setTransitionMatrix(false);
                    this->center_state_to_armor_measure.setMeasuredArmorIndex(0);
                    Eigen::Vector3d xyz_1 = poseToXYZ(indexed_poses.at(1).second.translate);
                    this->center_state_filter->predict(center_state_to_armor_measure, xyz_1);
                    this->filtered_center_state = this->center_state_filter->update();

                    this->center_state_to_armor_measure.inputYawAngle(yaw_2); 
                    setProcessNoise(true);  
                    setTransitionMatrix(true);
                    this->center_state_to_armor_measure.setMeasuredArmorIndex(1);
                    Eigen::Vector3d xyz_2 = poseToXYZ(indexed_poses.at(0).second.translate);
                    this->center_state_filter->predict(center_state_to_armor_measure, xyz_2);
                    this->filtered_center_state = this->center_state_filter->update();
                }
            }
        }
        else // 单装甲板观测
        {
            Eigen::Vector3d xyz = poseToXYZ(indexed_poses.at(0).second.translate);
            double distance = xyz.norm();
            setMeasurementNoise(distance);

            double yaw_this_time;
            if((indexed_poses.at(0).first == 0) || (indexed_poses.at(0).first == 2))
            {
                if(indexed_poses.at(0).first == 0)
                {
                    yaw_this_time = two_yaw[0];
                }
                else
                {
                    yaw_this_time = two_yaw[0] + M_PI;
                }
                this->center_state_to_armor_measure.setMeasuredArmorIndex(0);
            }
            else
            {
                if(indexed_poses.at(0).first == 1)
                {
                    yaw_this_time = two_yaw[1];
                }
                else
                {
                    yaw_this_time = two_yaw[1] + M_PI;
                }
                this->center_state_to_armor_measure.setMeasuredArmorIndex(1);
            }

            this->center_state_to_armor_measure.inputClearance(this->clearance);
            this->center_state_to_armor_measure.inputYawAngle(yaw_this_time);
            setProcessNoise(NO_DOUBLE_MEASURE);
            setTransitionMatrix(NO_DOUBLE_MEASURE);
            this->center_state_filter->predict(center_state_to_armor_measure, xyz);
            this->filtered_center_state = this->center_state_filter->update();
        }
    };

    void CenterStateEstimator::setTransitionMatrix(bool is_double_measure)
    {
        if(is_double_measure)
        {
            Eigen::Matrix<double, 8, 8> t_matrix = Eigen::Matrix<double, 8, 8>::Zero();
            t_matrix.diagonal() << 1, 1, 1, 1, 1, 1, 1, 1;
            this->center_state_filter->transition_matrix = t_matrix;
        }
        else
        {
            this->center_state_filter->transition_matrix<<  1, update_time, 0, 0, 0, 0, 0, 0,
                                                            0, 1, 0, 0, 0, 0, 0, 0,
                                                            0, 0, 1, update_time, 0, 0, 0, 0,
                                                            0, 0, 0, 1, 0, 0, 0, 0,
                                                            0, 0, 0, 0, 1, 0, update_time, 0,
                                                            0, 0, 0, 0, 0, 1, update_time, 0,
                                                            0, 0, 0, 0, 0, 0, 1, 0,
                                                            0, 0, 0, 0, 0, 0, 0, 1;
        }

    }

    void CenterStateEstimator::setProcessNoise(bool is_double_measure)
    {
        if(is_double_measure)
        {
            this->center_state_filter->process_noise_cov = Eigen::Matrix<double,8,8>::Zero();
        }
        else
        {
            Eigen::Matrix<double, 2, 1> process_noice_vec;
            process_noice_vec << 0.5 * update_time * update_time, update_time;
            Eigen::Matrix<double,1,1> process_noice_x;
            process_noice_x.diagonal() << X_PROCESS_NOICE;
            Eigen::Matrix<double,1,1> process_noice_y;
            process_noice_y.diagonal() << Y_PROCESS_NOICE;
            Eigen::Matrix<double,1,1> process_noice_z;
            process_noice_z.diagonal() << Z_PROCESS_NOICE;
            Eigen::Matrix2d process_noice_matrix_x = process_noice_vec * process_noice_x * process_noice_vec.transpose();
            Eigen::Matrix2d process_noice_matrix_y = process_noice_vec * process_noice_y * process_noice_vec.transpose();
            Eigen::Matrix2d process_noice_matrix_z = process_noice_vec * process_noice_z * process_noice_vec.transpose();
            Eigen::Matrix3d process_noice_matrix_double_z;
            process_noice_matrix_double_z << process_noice_matrix_z(0,0), 0, process_noice_matrix_z(0,1),
                                            0, process_noice_matrix_z(0,0), process_noice_matrix_z(0,1),
                                            process_noice_matrix_z(1,0), process_noice_matrix_z(1,0), process_noice_matrix_z(1,1);
            this->center_state_filter->process_noise_cov.block<2,2>(0,0) = process_noice_matrix_x;
            
            this->center_state_filter->process_noise_cov.block<2,2>(2,2) = process_noice_matrix_y;
            this->center_state_filter->process_noise_cov.block<3,3>(4,4) = process_noice_matrix_double_z;
        }
    }

    void CenterStateEstimator::setMeasurementNoise(double distance)
    {
        // TODO : 随距离调大测量噪声 
        this->center_state_filter->measurement_cov_maxtrix.diagonal() << X_MEASURE_NOICE, Y_MEASURE_NOICE, Z_MEASURE_NOICE;
    }

   Eigen::Vector3d CenterStateEstimator::poseToXYZ(Eigen::Vector3d pose)
   {
        // 需要交换[1]和[2]
        return {pose[0], pose[2], -pose[1]};
   }

   void CenterStateEstimator::rebootEstimator(SOLVER::IndexedArmorPoses indexed_armor_poses)
   {
        this->center_state_filter->error_cov_post = Eigen::Matrix<double, 8, 8>::Identity();
        this->clearance_filter->error_cov_post = Eigen::Matrix<double, 1, 1>::Identity();

        // clearance_filter初始化
        this->clearance_filter->transition_matrix << 1;
        this->clearance_filter->process_noise_cov << CLEARANCE_POCESS_NOICE;

        double x_a,y_a,z_a,k;
        if(indexed_armor_poses.size() == 1)
        {
            Eigen::Vector3d xyz_ = poseToXYZ(indexed_armor_poses.at(0).second.translate);
            x_a = xyz_[0];
            y_a = xyz_[1];
            z_a = xyz_[2];
        }
        else
        {
            Eigen::Vector3d xyz_1 = poseToXYZ(indexed_armor_poses.at(0).second.translate);
            x_a = xyz_1[0];
            y_a = xyz_1[1];
            z_a = xyz_1[2];
        }
        
        this->center_state_filter->posteriori_state_estimate = {x_a,0,y_a,0,z_a,z_a,0,0.414};
   }

    Eigen::Vector4d CenterStateEstimator::getCarCenterPosition()
    {
        auto center_state = umt::ObjManager<Eigen::Matrix<double, 8, 1>>::find_or_create("center_state");
        *center_state.get() = center_state_filter->posteriori_state_estimate;
        // std::cout<<"state :"<<center_state_filter->posteriori_state_estimate<<std::endl;
        return {this->center_state_filter->posteriori_state_estimate[0],this->center_state_filter->posteriori_state_estimate[2],this->center_state_filter->posteriori_state_estimate[4],this->center_state_filter->posteriori_state_estimate[5]};
    }

    Eigen::Vector3d CenterStateEstimator::getCarCenterSpeed()
    {
        return {this->center_state_filter->posteriori_state_estimate[1],this->center_state_filter->posteriori_state_estimate[3],this->center_state_filter->posteriori_state_estimate[6]};
    }

    double CenterStateEstimator::getRadiusProportion()
    {
        return this->center_state_filter->posteriori_state_estimate[7];
    }

    double CenterStateEstimator::getArmorClearance()
    {
        std::cout<<"clearance: "<<clearance<<std::endl;
        return this->clearance;
    }

} // namespace PREDICTOR
