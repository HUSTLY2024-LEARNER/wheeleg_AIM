#pragma once

#include "EKF.hpp"
#include "solver.hpp"
#include <memory>

#define CLEARANCE_POCESS_NOICE 0.001

#define X_PROCESS_NOICE 50
#define Y_PROCESS_NOICE 50
#define Z_PROCESS_NOICE 20

#define X_MEASURE_NOICE 0.01
#define Y_MEASURE_NOICE 0.01
#define Z_MEASURE_NOICE 0.01

namespace PREDICTOR
{
    class CenterState2ArmorMeasure
    {
    public:
        template <class T>
        void operator()(const T center_state[8], T armor_measure[3])
        {
            if(armor_index == 0)
            {
                // x_a
                armor_measure[0] = center_state[0] - clearance * center_state[7] * sin(yaw_angle);
                // y_a
                armor_measure[1] = center_state[2] - clearance * center_state[7] * cos(yaw_angle);
                // z_a
                armor_measure[2] = center_state[4]; 
            }
            else
            {
                //泰勒展开：sqrt(1-x^2) 约等于 1-x^2/2-x^4/(2*4)
                // x_a
                armor_measure[0] = center_state[0] - clearance * (1.0 - center_state[7] * center_state[7] / 2.0 - center_state[7] * center_state[7] * center_state[7] * center_state[7] / 8.0) * sin(yaw_angle);
                // y_a
                armor_measure[1] = center_state[2] - clearance * (1.0 - center_state[7] * center_state[7] / 2.0 - center_state[7] * center_state[7] * center_state[7] * center_state[7] / 8.0) * cos(yaw_angle);
                // z_a
                armor_measure[2] = center_state[5]; 
            }

        }
        void inputClearance(double clearance_){this->clearance = clearance_;};
        void inputYawAngle(double angle_){this->yaw_angle = angle_;};
        void setMeasuredArmorIndex(int index_){this->armor_index = index_;};
    private:
        double clearance;
        int armor_index;
        double yaw_angle;
    };

    class ClearanceState2ClearanceMeasure
    {
    public:
        template <class T>
        void operator()(const T clearance_state[1], T clearance_measure[1])
        {
            clearance_measure[0] = clearance_state[0];
        }
    };

    // 根据装甲板观测的x,y,z,yaw和radiusEstimator滤波出的半径结果得出车辆中心的平移速度
    class CenterStateEstimator
    {
    public:
        CenterStateEstimator();
        ~CenterStateEstimator();
        void inputMeasurement(SOLVER::IndexedArmorPoses,Eigen::Vector2d);
        Eigen::Vector4d getCarCenterPosition();
        Eigen::Vector3d getCarCenterSpeed(); 
        double getRadiusProportion();
        double getArmorClearance();
        CenterState2ArmorMeasure center_state_to_armor_measure;
        void rebootEstimator(SOLVER::IndexedArmorPoses);
        void setT(double);
    private:
        // (x_c,v_x,y_c,v_y,z_1,z_2,v_z,k),(x_a,y_a,z_a)
        std::unique_ptr<ExtendedKalman<double,8,3>> center_state_filter;
        Eigen::Matrix<double,8,1> filtered_center_state;
        Eigen::Vector3d poseToXYZ(Eigen::Vector3d);
        void setTransitionMatrix(bool);
        void setProcessNoise(bool);
        void setMeasurementNoise(double);
        int last_single_index;
        double update_time;

        // (l),(l)
        std::unique_ptr<ExtendedKalman<double,1,1>> clearance_filter;
        ClearanceState2ClearanceMeasure clearance_state_to_measure;
        double clearance;
        double doublePoseToClearance(SOLVER::ArmorPose,SOLVER::ArmorPose);

    };

}
