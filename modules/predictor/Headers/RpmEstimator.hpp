#pragma once

#include "EKF.hpp"
#include "solver.hpp"
#include <memory>

#define YAW_PROCESS_NOICE 1000

#define CLOCK_WISE 1
#define COUNTER_CLOCK_WISE -1
#define UNKNOW 0

#define MIN_WINDOW_SIZE 3

namespace PREDICTOR
{
    class YawState2YawMeasure
    {
    public:
        template <class T>
        void operator()(const T yaw_state[3], T yaw_measure[3])
        {
            yaw_measure[0] = yaw_state[0];
            yaw_measure[1] = yaw_state[1];
            yaw_measure[2] = yaw_state[1] - yaw_state[0];
        }
    };

    // 根据距离选择使用角度估计转速还是滑动窗口内装甲板切换频率估计转速
    // 当使用欧拉角滤波出来的角速度大于某个值，认为对面在小陀螺时，才创建滑动窗口
    class RpmEstimator
    {
    public:
        RpmEstimator();
        ~RpmEstimator();
        void inputMeasurement(SOLVER::IndexedArmorPoses);
        double getAngularSpeed(){return angular_speed;};
        std::pair<int,double> getStatisticalAngularState(){return std::make_pair(spin_state,statistical_angular_speed);};
        Eigen::Vector2d getDoubleAngle();
        double getArmorSwitchFrequency(int);
        void rebootEstimator(SOLVER::IndexedArmorPoses);
        void setT(double);
        YawState2YawMeasure yaw_state_to_yaw_measure;
        int getSpinState();
        
    private:
        // (theta_1,theta_2,omega), (theta_1,theta_2,PI/2)
        std::unique_ptr<ExtendedKalman<double,3,3>> yaw_filter;
        double calculateSingleYaw(Sophus::SE3<double>);
        std::pair<double,double> calculateDoubleYaw(Sophus::SE3<double>,Sophus::SE3<double>); 
        Eigen::Vector3d makeYawMeasurement(double,int);
        Eigen::Vector3d makeYawMeasurement(double,int,double,int);
        void setTransitionMatrix();
        void setProcessNoise();
        void setMeasurementNoise(double,bool);
        Eigen::Vector3d makeYawContinues(Eigen::Vector3d);
        double angleFusion(double,double);
        double angular_speed;
        double statistical_angular_speed;
        double update_time;
        double last_yaw;
        int last_cloest_index;
        int spin_state;
        std::vector<std::chrono::time_point<std::chrono::system_clock>> time_stamp_window;
    };
}
