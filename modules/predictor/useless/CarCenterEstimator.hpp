// #pragma once

// #include "EKF.hpp"
// #include "solver.hpp"
// #include <memory>

// #define X_PROCESS_NOICE 100
// #define Y_PROCESS_NOICE 100
// #define Z_PROCESS_NOICE 100

// #define X_MEASURE_NOICE 1
// #define Y_MEASURE_NOICE 1
// #define Z_MEASURE_NOICE 1

// namespace PREDICTOR
// {
//     class CenterState2ArmorMeasure
//     {
//     public:
//         template <class T>
//         void operator()(const T center_state[7], T armor_measure[3])
//         {
//             // x_a
//             armor_measure[0] = center_state[0] - radius * sin(yaw_angle);
//             // y_a
//             armor_measure[1] = center_state[1] + radius * cos(yaw_angle);
//             // z_a
//             if(z_index == 0)
//             {
//                 armor_measure[2] = center_state[4]; 
//             }
//             else
//             {
//                 armor_measure[2] = center_state[5]; 
//             }

//         }
//         void inputRadius(double radius_){this->radius = radius_;};
//         void inputYawAngle(double angle_){this->yaw_angle = angle_;};
//         void setMeasuredZ(int index_){this->z_index = index_;};
//     private:
//         double radius;
//         int z_index;
//         double yaw_angle;
//     };

//     // 根据装甲板观测的x,y,z,yaw和radiusEstimator滤波出的半径结果得出车辆中心的平移速度
//     class CarCenterEstimator
//     {
//     public:
//         CarCenterEstimator();
//         ~CarCenterEstimator();
//         void inputMeasurement(SOLVER::IndexedArmorPoses,Eigen::Vector2d,Eigen::Vector2d);
//         Eigen::Vector3d getCarCenterPosition();
//         Eigen::Vector3d getCarCenterSpeed(); 
//         CenterState2ArmorMeasure center_state_to_armor_measure;
//         void rebootEstimamtor();
//     private:
//         // (x_c,v_x,y_c,v_y,z_1,z_2,v_z),(x_a,y_a,z_a)
//         std::unique_ptr<ExtendedKalman<double,7,3>> car_center_filter;
//         Eigen::Matrix<double,7,1> filtered_center_state;
//         Eigen::Vector3d poseToXYZ(Sophus::SE3<double>);
//         void setTransitionMatrix(bool);
//         void setProcessNoise(bool);
//         void setMeasurementNoise(double);
//         int last_single_index;
//         double update_time;
//     };
// }