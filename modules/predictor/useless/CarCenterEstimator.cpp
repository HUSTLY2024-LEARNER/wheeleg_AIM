// #include "CarCenterEstimator.hpp"

// namespace PREDICTOR
// {
//     CarCenterEstimator::CarCenterEstimator()
//     {
//         this->car_center_filter = std::make_unique<ExtendedKalman<double,7,3>>();
//     };

//     CarCenterEstimator::~CarCenterEstimator()
//     {

//     };

//     void CarCenterEstimator::inputMeasurement(SOLVER::IndexedArmorPoses indexed_poses, Eigen::Vector2d two_radius, Eigen::Vector2d two_yaw)
//     {
//         double distance = 1;
//         setMeasurementNoise(distance);
//         // 需要得到的信息有：目标的x,y,z和yaw(读rpm estimator滤出来的角度)
//         if(indexed_poses.size() == 1) // 只有单装甲板观测
//         {
//             Eigen::Vector3d xyz = indexed_poses.at(0).second.translate;
//             double radius_this_time, yaw_this_time;
//             if((indexed_poses.at(0).first == 0) || (indexed_poses.at(0).first == 2))
//             {
//                 radius_this_time = two_radius[0];
//                 if(indexed_poses.at(0).first == 0)
//                 {
//                     yaw_this_time = two_yaw[0];
//                 }
//                 else
//                 {
//                     yaw_this_time = two_yaw[0] + M_PI;
//                 }
//                 this->center_state_to_armor_measure.setMeasuredZ(0);
//             }
//             else
//             {
//                 radius_this_time = two_radius[1];
//                 if(indexed_poses.at(0).first == 1)
//                 {
//                     yaw_this_time = two_yaw[1];
//                 }
//                 else
//                 {
//                     yaw_this_time = two_yaw[1] + M_PI;
//                 }
//                 this->center_state_to_armor_measure.setMeasuredZ(1);
//             }
//             this->center_state_to_armor_measure.inputRadius(radius_this_time);
//             this->center_state_to_armor_measure.inputYawAngle(yaw_this_time);
//             setProcessNoise(false);
//             setTransitionMatrix(false);
//             this->car_center_filter->predict(center_state_to_armor_measure, xyz);
//             this->filtered_center_state = this->car_center_filter->update();
//         }
//         else // 双装甲板观测
//         {
//             if((indexed_poses.at(0).first - indexed_poses.at(1).first == 1) || (indexed_poses.at(0).first - indexed_poses.at(1).first == -3))
//             {
//                 if(indexed_poses.at(1).first == 0 || indexed_poses.at(1).first == 2) // (1)对应r_0，(0)对应r_1
//                 {
//                     double yaw_1, yaw_2;
//                     if(indexed_poses.at(1).first == 0)
//                     {
//                         yaw_1 = two_yaw[0];
//                         yaw_2 = two_yaw[1];
//                     }
//                     else
//                     {
//                         yaw_1 = two_yaw[0] + M_PI;
//                         yaw_2 = two_yaw[1] + M_PI;
//                     }
//                     // 先用(1)更新滤波器，修改状态转移函数和process_noise后再用(0)更新滤波器
//                     this->center_state_to_armor_measure.inputRadius(two_radius[0]); 
//                     this->center_state_to_armor_measure.inputYawAngle(yaw_1); 
//                     setProcessNoise(false);  
//                     setTransitionMatrix(false);
//                     this->center_state_to_armor_measure.setMeasuredZ(0);
//                     Eigen::Vector3d xyz_1 = indexed_poses.at(1).second.translate;
//                     this->car_center_filter->predict(center_state_to_armor_measure, xyz_1);
//                     this->filtered_center_state = this->car_center_filter->update();

//                     this->center_state_to_armor_measure.inputRadius(two_radius[1]); 
//                     this->center_state_to_armor_measure.inputYawAngle(yaw_2); 
//                     setProcessNoise(true);  
//                     setTransitionMatrix(true);
//                     this->center_state_to_armor_measure.setMeasuredZ(1);
//                     Eigen::Vector3d xyz_2 = indexed_poses.at(0).second.translate;
//                     this->car_center_filter->predict(center_state_to_armor_measure, xyz_2);
//                     this->filtered_center_state = this->car_center_filter->update();
//                 }
//                 else // (0)对应r_0，(1)对应r_1
//                 {
//                     double yaw_1, yaw_2;
//                     if(indexed_poses.at(0).first == 0)
//                     {
//                         yaw_1 = two_yaw[0];
//                         yaw_2 = two_yaw[1] + M_PI;
//                     }
//                     else
//                     {
//                         yaw_1 = two_yaw[0] + M_PI;
//                         yaw_2 = two_yaw[1];
//                     }
//                     // 先用(1)更新滤波器，修改状态转移函数和process_noise后再用(0)更新滤波器
//                     this->center_state_to_armor_measure.inputRadius(two_radius[0]); 
//                     this->center_state_to_armor_measure.inputYawAngle(yaw_1); 
//                     setProcessNoise(false);  
//                     setTransitionMatrix(false);
//                     this->center_state_to_armor_measure.setMeasuredZ(0);
//                     Eigen::Vector3d xyz_1 = indexed_poses.at(0).second.translate;
//                     this->car_center_filter->predict(center_state_to_armor_measure, xyz_1);
//                     this->filtered_center_state = this->car_center_filter->update();

//                     this->center_state_to_armor_measure.inputRadius(two_radius[1]); 
//                     this->center_state_to_armor_measure.inputYawAngle(yaw_2); 
//                     setProcessNoise(true);  
//                     setTransitionMatrix(true);
//                     this->center_state_to_armor_measure.setMeasuredZ(1);
//                     Eigen::Vector3d xyz_2 = indexed_poses.at(1).second.translate;
//                     this->car_center_filter->predict(center_state_to_armor_measure, xyz_2);
//                     this->filtered_center_state = this->car_center_filter->update();
//                 }
//             }
//             else if((indexed_poses.at(0).first - indexed_poses.at(1).first == -1) || (indexed_poses.at(0).first - indexed_poses.at(1).first == 3))
//             {
//                 if(indexed_poses.at(0).first == 0 || indexed_poses.at(0).first == 2) // (0)对应r_0，(1)对应r_1
//                 {
//                     double yaw_1, yaw_2;
//                     if(indexed_poses.at(0).first == 0)
//                     {
//                         yaw_1 = two_yaw[0];
//                         yaw_2 = two_yaw[1];
//                     }
//                     else
//                     {
//                         yaw_1 = two_yaw[0] + M_PI;
//                         yaw_2 = two_yaw[1] + M_PI;
//                     }
//                     // 先用(1)更新滤波器，修改状态转移函数和process_noise后再用(0)更新滤波器
//                     this->center_state_to_armor_measure.inputRadius(two_radius[0]); 
//                     this->center_state_to_armor_measure.inputYawAngle(yaw_1); 
//                     setProcessNoise(false);  
//                     setTransitionMatrix(false);
//                     this->center_state_to_armor_measure.setMeasuredZ(0);
//                     Eigen::Vector3d xyz_1 = indexed_poses.at(0).second.translate;
//                     this->car_center_filter->predict(center_state_to_armor_measure, xyz_1);
//                     this->filtered_center_state = this->car_center_filter->update();

//                     this->center_state_to_armor_measure.inputRadius(two_radius[1]); 
//                     this->center_state_to_armor_measure.inputYawAngle(yaw_2); 
//                     setProcessNoise(true);  
//                     setTransitionMatrix(true);
//                     this->center_state_to_armor_measure.setMeasuredZ(1);
//                     Eigen::Vector3d xyz_2 = indexed_poses.at(1).second.translate;
//                     this->car_center_filter->predict(center_state_to_armor_measure, xyz_2);
//                     this->filtered_center_state = this->car_center_filter->update();
//                 }
//                 else // (1)对应r_0，(0)对应r_1
//                 {
//                     double yaw_1, yaw_2;
//                     if(indexed_poses.at(1).first == 0)
//                     {
//                         yaw_1 = two_yaw[0];
//                         yaw_2 = two_yaw[1] + M_PI;
//                     }
//                     else
//                     {
//                         yaw_1 = two_yaw[0] + M_PI;
//                         yaw_2 = two_yaw[1];
//                     }
//                     // 先用(1)更新滤波器，修改状态转移函数和process_noise后再用(0)更新滤波器
//                     this->center_state_to_armor_measure.inputRadius(two_radius[0]); 
//                     this->center_state_to_armor_measure.inputYawAngle(yaw_1); 
//                     setProcessNoise(false);  
//                     setTransitionMatrix(false);
//                     this->center_state_to_armor_measure.setMeasuredZ(0);
//                     Eigen::Vector3d xyz_1 = indexed_poses.at(1).second.translate;
//                     this->car_center_filter->predict(center_state_to_armor_measure, xyz_1);
//                     this->filtered_center_state = this->car_center_filter->update();

//                     this->center_state_to_armor_measure.inputRadius(two_radius[1]); 
//                     this->center_state_to_armor_measure.inputYawAngle(yaw_2); 
//                     setProcessNoise(true);  
//                     setTransitionMatrix(true);
//                     this->center_state_to_armor_measure.setMeasuredZ(1);
//                     Eigen::Vector3d xyz_2 = indexed_poses.at(0).second.translate;
//                     this->car_center_filter->predict(center_state_to_armor_measure, xyz_2);
//                     this->filtered_center_state = this->car_center_filter->update();
//                 }
//             }
//         }
//     };

//     void CarCenterEstimator::setTransitionMatrix(bool is_double_measure)
//     {
//         if(is_double_measure)
//         {
//             Eigen::Matrix<double,7,7> t_matrix = Eigen::Matrix<double,7,7>::Zero();
//             t_matrix.diagonal() <<1, 1, 1, 1, 1, 1, 1;
//             this->car_center_filter->transition_matrix = t_matrix;
//         }
//         else
//         {
//             this->car_center_filter->transition_matrix<<1, update_time, 0, 0, 0, 0, 0,
//                                             0, 1, 0, 0, 0, 0, 0,
//                                             0, 0, 1, update_time, 0, 0, 0,
//                                             0, 0, 0, 1, 0, 0, 0,
//                                             0, 0, 0, 0, 1, 0, update_time,
//                                             0, 0, 0, 0, 0, 1, update_time,
//                                             0, 0, 0, 0, 0, 0, 1;
//         }

//     }

//     void CarCenterEstimator::setProcessNoise(bool is_double_measure)
//     {
//         if(is_double_measure)
//         {
//             this->car_center_filter->process_noise_cov = Eigen::Matrix<double,7,7>::Zero();
//         }
//         else
//         {
//             Eigen::Matrix<double, 2, 1> process_noice_vec;
//             process_noice_vec << 0.5 * update_time * update_time, update_time;
//             Eigen::Matrix<double,1,1> process_noice_x;
//             process_noice_x.diagonal() << X_PROCESS_NOICE;
//             Eigen::Matrix<double,1,1> process_noice_y;
//             process_noice_y.diagonal() << X_PROCESS_NOICE;
//             Eigen::Matrix<double,1,1> process_noice_z;
//             process_noice_z.diagonal() << X_PROCESS_NOICE;
//             Eigen::Matrix2d process_noice_matrix_x = process_noice_vec * process_noice_x * process_noice_vec.transpose();
//             Eigen::Matrix2d process_noice_matrix_y = process_noice_vec * process_noice_y * process_noice_vec.transpose();
//             Eigen::Matrix2d process_noice_matrix_z = process_noice_vec * process_noice_z * process_noice_vec.transpose();
//             Eigen::Matrix3d process_noice_matrix_double_z;
//             process_noice_matrix_double_z << process_noice_matrix_z(0,0), 0, process_noice_matrix_z(0,1),
//                                             0, process_noice_matrix_z(0,0), process_noice_matrix_z(0,1),
//                                             process_noice_matrix_z(1,0), process_noice_matrix_z(1,0), process_noice_matrix_z(1,1);
//             this->car_center_filter->process_noise_cov.block<2,2>(0,0) = process_noice_matrix_x;
            
//             this->car_center_filter->process_noise_cov.block<2,2>(2,2) = process_noice_matrix_y;
//             this->car_center_filter->process_noise_cov.block<3,3>(4,4) = process_noice_matrix_double_z;
//         }
//     }

//     void CarCenterEstimator::setMeasurementNoise(double distance)
//     {
//         // TODO : 随距离调大测量噪声 
//         this->car_center_filter->measurement_cov_maxtrix.diagonal() << X_MEASURE_NOICE, Y_MEASURE_NOICE, Z_MEASURE_NOICE;
//     }

//    Eigen::Vector3d CarCenterEstimator::poseToXYZ(Sophus::SE3<double> pose)
//    {
//         return pose.translation();
//    }

//    void CarCenterEstimator::rebootEstimamtor()
//    {
    
//    }

//     Eigen::Vector3d CarCenterEstimator::getCarCenterPosition()
//     {
//         return {this->car_center_filter->posteriori_state_estimate[0],this->car_center_filter->posteriori_state_estimate[2],this->car_center_filter->posteriori_state_estimate[4]};
//     }

//     Eigen::Vector3d CarCenterEstimator::getCarCenterSpeed()
//     {
//         return {this->car_center_filter->posteriori_state_estimate[1],this->car_center_filter->posteriori_state_estimate[3],this->car_center_filter->posteriori_state_estimate[6]};
//     }

// } // namespace PREDICTOR
