// #pragma once

// #include "EKF.hpp"
// #include <memory>
// #include "solver.hpp"

// #define DISTANCE_PROCESS_NOICE 0.0001
// #define RADIUS_PROCESS_NOICE 0.0001

// #define DISTANCE_MEASURE_NOICE 0.1
// #define ANGLE_MEASURE_NOICE 4
// #define PERFECT_MEASURE_NOICE 0.0000001

// namespace PREDICTOR
// {
//     class RadiusState2RadiusMeasure
//     {
//     public:
//         template <class T>
//         void operator()(const T radius_state[3], T radius_measure[4])
//         {
//             radius_measure[0] = radius_state[0];
//             radius_measure[1] = radius_state[1];
//             radius_measure[2] = radius_state[2];
//             radius_measure[3] = radius_state[0] * radius_state[0] - radius_state[1] * radius_state[1] - radius_state[2] * radius_state[2];
//         }
//     };
    
//     // 当同时观测到两块装甲板或者装甲板发生切换的时候有一个条件应该始终满足：
//     // 算出的车辆中心不应该发生跳变，故可以直接求解（或迭代求解出两个半径）
//     // 将计算得出的半径放入滤波器中，这是个非时间相关的滤波器
//     class RadiusEstimator
//     {
//     public:
//         RadiusEstimator();
//         ~RadiusEstimator();
//         void inputMeasurement(SOLVER::IndexedArmorPoses);
//         double getArmorClearance(){return this->radius_state[0];};
//         Eigen::Vector2d getRadius(){return {this->radius_state[0], this->radius_state[1]};};
//         void rebootEstimator();
//         void setDoubleYaw(Eigen::Vector2d double_yaw_){this->double_yaw = double_yaw_;};
//         RadiusState2RadiusMeasure radius_state_to_measure;
//     private:
//         // (distance, r_1, r_2), (distance，r_1, r_2, 0)
//         std::unique_ptr<ExtendedKalman<double,3,4>> radius_filter;
//         Eigen::Vector<double,4> makeRadiusMeasurement(SOLVER::ArmorPose, int, SOLVER::ArmorPose, int);
//         void setMeasurementNoise(double);
//         void setProcessNoise();
//         void setTransitionMatrix();
//         Eigen::Vector3d radius_state = {0.283, 0.2, 0.2};
//         int last_single_index;
//         SOLVER::ArmorPose last_single_pose;
//         Eigen::Vector2d double_yaw;
//     };
// }