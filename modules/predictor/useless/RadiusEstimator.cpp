// #include "RadiusEstimator.hpp"
// #include "solver.hpp"

// namespace PREDICTOR
// {
//     RadiusEstimator::RadiusEstimator()
//     {
//         this->radius_filter = std::make_unique<ExtendedKalman<double,3,4>>();
//         setProcessNoise();
//         setTransitionMatrix();
//         last_single_index = 999;
//     };

//     RadiusEstimator::~RadiusEstimator()
//     {

//     };

//     void RadiusEstimator::inputMeasurement(SOLVER::IndexedArmorPoses indexed_poses)
//     {
//         // 需要从pose中得到的信息有：
//         // 1.当观测到两块装甲板时，可以求解出两个半径；
//         // 2.当发生装甲板切换时，也可以求解出两个半径

//         setMeasurementNoise(indexed_poses.at(0).second.translate.norm());

//         if(indexed_poses.size() == 1) // 只有单装甲板观测
//         {
//             // 需要判断是否发生装甲板切换，若发生装甲板切换，生成半径观测数据
//             int index_now = indexed_poses.at(0).first;
//             if((index_now - last_single_index == 1) || (index_now - last_single_index == -3))
//             {
//                 COUT("ARMOR SWITCH", RED);
//                 Eigen::Vector<double,4> radius_measure = makeRadiusMeasurement(indexed_poses.at(0).second, index_now, last_single_pose, last_single_index);
//                 this->radius_filter->predict(radius_state_to_measure, radius_measure);
//                 this->radius_state = this->radius_filter->update();
//                 last_single_index = index_now;
//                 last_single_pose = indexed_poses.at(0).second;
//             }
//             else
//             {
//                 COUT("ARMOR NOT SWITCH", RED);
//                 last_single_index = index_now;
//                 last_single_pose = indexed_poses.at(0).second;
//             }
//         }
//         else // 有双装甲板观测
//         {
//             COUT("DOUBLE ARMOR OBSERVE", RED);
//             Eigen::Vector<double,4> radius_measure = makeRadiusMeasurement(indexed_poses.at(0).second, indexed_poses.at(0).first, indexed_poses.at(1).second, indexed_poses.at(1).first);
//             this->radius_filter->predict(radius_state_to_measure, radius_measure);
//             this->radius_state = this->radius_filter->update();
//         }
//     }
    
//     Eigen::Vector<double,4> RadiusEstimator::makeRadiusMeasurement(SOLVER::ArmorPose armor_pose_1, int index_1, SOLVER::ArmorPose armor_pose_2, int index_2)
//     {
//         // 先求distance
//         double distance = std::sqrt(std::pow((armor_pose_1.translate.x() - armor_pose_2.translate.x()), 2) + std::pow((armor_pose_1.translate.z() - armor_pose_2.translate.z()), 2));
        
//         double x_1, y_1, theta_1, x_2, y_2, theta_2;
            
//         if(index_1 == 0 && index_2 == 1)
//         {
//             x_1 = armor_pose_1.translate.x();
//             y_1 = armor_pose_1.translate.z();
//             theta_1 = this->double_yaw[0];
//             x_2 = armor_pose_2.translate.x();
//             y_2 = armor_pose_2.translate.z();
//             theta_2 = this->double_yaw[1] - M_PI;
//         }
//         else if(index_1 == 1 && index_2 == 2)
//         {
//             x_2 = armor_pose_1.translate.x();
//             y_2 = armor_pose_1.translate.z();
//             theta_2 = this->double_yaw[0];
//             x_1 = armor_pose_2.translate.x();
//             y_1 = armor_pose_2.translate.z();
//             theta_1 = this->double_yaw[1];
//         }
//         else if(index_1 == 2 && index_2 ==3)
//         {
//             x_1 = armor_pose_1.translate.x();
//             y_1 = armor_pose_1.translate.z();
//             theta_1 = this->double_yaw[0] - M_PI;
//             x_2 = armor_pose_2.translate.x();
//             y_2 = armor_pose_2.translate.z();
//             theta_2 = this->double_yaw[1];
//         }
//         else if(index_1 == 3 && index_2 == 0)
//         {
//             x_2 = armor_pose_1.translate.x();
//             y_2 = armor_pose_1.translate.z();
//             theta_2 = this->double_yaw[0] - M_2_PI;
//             x_1 = armor_pose_2.translate.x();
//             y_1 = armor_pose_2.translate.z();
//             theta_1 = this->double_yaw[1] - M_PI - M_2_PI;
//         }
//         else if(index_2 == 0 && index_1 == 1)
//         {
//             x_2 = armor_pose_1.translate.x();
//             y_2 = armor_pose_1.translate.z();
//             theta_2 = this->double_yaw[0];
//             x_1 = armor_pose_2.translate.x();
//             y_1 = armor_pose_2.translate.z();
//             theta_1 = this->double_yaw[1] - M_PI;
//         }
//         else if(index_2 == 1 && index_1 == 2)
//         {
//             x_1 = armor_pose_1.translate.x();
//             y_1 = armor_pose_1.translate.z();
//             theta_1 = this->double_yaw[0] - M_2_PI;
//             x_2 = armor_pose_2.translate.x();
//             y_2 = armor_pose_2.translate.z();
//             theta_2 = this->double_yaw[1] - M_PI - M_2_PI;
//         }
//         else if(index_2 == 2 && index_1 ==3)
//         {
//             x_2 = armor_pose_1.translate.x();
//             y_2 = armor_pose_1.translate.z();
//             theta_2 = this->double_yaw[0] - M_PI;
//             x_1 = armor_pose_2.translate.x();
//             y_1 = armor_pose_2.translate.z();
//             theta_1 = this->double_yaw[1];
//         }
//         else if(index_2 == 3 && index_1 == 0)
//         {
//             x_1 = armor_pose_1.translate.x();
//             y_1 = armor_pose_1.translate.z();
//             theta_1 = this->double_yaw[0] - M_2_PI;
//             x_2 = armor_pose_2.translate.x();
//             y_2 = armor_pose_2.translate.z();
//             theta_2 = this->double_yaw[1] - M_PI - M_2_PI;
//         }

        

//         std::cout<<"x_1: "<<x_1<<std::endl;
//         std::cout<<"y_1: "<<y_1<<std::endl;
//         std::cout<<"theta_1: "<<theta_1<<std::endl;
//         std::cout<<"x_2: "<<x_2<<std::endl;
//         std::cout<<"y_2: "<<y_2<<std::endl;
//         std::cout<<"theta_2: "<<theta_2<<std::endl;

//         double r_1 = (((x_2 - x_1) / sin(theta_2)) + ((y_1 - y_2) / cos(theta_2))) / ((sin(theta_1) / sin(theta_2)) - (cos(theta_1) / cos(theta_2)));
//         double r_2 = (((x_1 - x_2) / sin(theta_1)) + ((y_2 - y_1) / cos(theta_1))) / ((sin(theta_2) / sin(theta_1)) - (cos(theta_2) / cos(theta_1)));

//         std::cout<<"k: "<< r_1 / r_2<<std::endl;

//         COUT("R_1: "<<r_1, RED);
//         COUT("R_2: "<<r_2, RED);

//         std::cout<<"x_c:"<<x_1 + r_1 * sin(theta_1);
//         std::cout<<"y_c:"<<y_1 + r_1 * cos(theta_1);

//         // 两装甲板间距、半径1、半径2、完美测量(保证勾股定理)
//         return {distance, r_1, r_2, 0};
//     }

//     void RadiusEstimator::setTransitionMatrix()
//     {
//         this->radius_filter->transition_matrix<< 1, 0, 0,
//                                                  0, 1, 0,
//                                                  0, 0, 1;
//     }

//     void RadiusEstimator::setProcessNoise()
//     {
//         this->radius_filter->process_noise_cov<<DISTANCE_PROCESS_NOICE, 0, 0,
//                                                 0, RADIUS_PROCESS_NOICE, 0,
//                                                 0, 0, RADIUS_PROCESS_NOICE;
//     }

//     void RadiusEstimator::setMeasurementNoise(double)
//     {
//         this->radius_filter->measurement_cov_maxtrix.diagonal() << DISTANCE_MEASURE_NOICE,ANGLE_MEASURE_NOICE,ANGLE_MEASURE_NOICE, PERFECT_MEASURE_NOICE;
//     }

//     void RadiusEstimator::rebootEstimator()
//     {

//     }
// }