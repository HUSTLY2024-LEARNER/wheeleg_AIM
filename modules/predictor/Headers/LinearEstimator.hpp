#pragma once

#include "EKF.hpp"
#include "solver.hpp"
#include <memory>

namespace PREDICTOR
{
    class LinearCAState2ArmorMeasure
    {
    public:
        template <class T>
        void operator()(const T linear_state[9], T armor_measure[3])
        {
            armor_measure[0] = linear_state[0];
            armor_measure[1] = linear_state[3];
            armor_measure[2] = linear_state[6];
        }
    };

    class LinearCVState2ArmorMeasure
    {
    public:
        template <class T>
        void operator()(const T linear_state[6], T armor_measure[3])
        {
            armor_measure[0] = linear_state[0];
            armor_measure[1] = linear_state[2];
            armor_measure[2] = linear_state[4];
        }
    };

    class LinearStaticState2ArmorMeasure
    {
    public:
        template <class T>
        void operator()(const T linear_state[3], T armor_measure[3])
        {
            armor_measure[0] = linear_state[0];
            armor_measure[1] = linear_state[1];
            armor_measure[2] = linear_state[2];
        }
    };

    class LinearCVState2PitchYawDistanceMeasure
    {
    public:
        template <class T>
        void operator()(const T linear_state[6], T pyd_measure[3])
        {
            pyd_measure[0] = ceres::atan2(linear_state[4], ceres::sqrt(linear_state[0] * linear_state[0] + linear_state[2] * linear_state[2])); // pitch
            pyd_measure[1] = ceres::atan2(linear_state[0], linear_state[2]);                                         // yaw
            pyd_measure[2] = ceres::sqrt(linear_state[0] * linear_state[0] + linear_state[4] * linear_state[4] + linear_state[2] * linear_state[2]);     // distance
        }
    };

    class LinearEstimator
    {
    public:
        LinearEstimator();
        ~LinearEstimator();
        void inputMeasurement(SOLVER::IndexedArmorPoses);
        void rebootEstimator(SOLVER::IndexedArmorPoses);
        void setT(double);

        LinearCAState2ArmorMeasure linear_ca_state_2_measure;
        LinearCVState2ArmorMeasure linear_cv_state_2_measure;
        LinearStaticState2ArmorMeasure linear_static_state_2_measure;
        LinearCVState2PitchYawDistanceMeasure linear_cv_state_2_pyd_measure;

        std::vector<Eigen::Vector3d> getMessageCA();
        std::vector<Eigen::Vector3d> getMessageCV();
        std::vector<Eigen::Vector3d> getMessageSphereCV();
        Eigen::Vector3d getMessageStatic();

        // input: CA, CV, STATIC  output: FUSION
        Eigen::Vector3d linearFusion(Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d);
    private:
        // (x,v_x,a_x,y,v_y,a_y,z,v_z,a_z), (x,y,z)
        std::unique_ptr<ExtendedKalman<double,9,3>> linear_filter_ca;
        // (x,v_x,y,v_y,z,v_z), (x,y,z)
        std::unique_ptr<ExtendedKalman<double,6,3>> linear_filter_cv;
        // (x,y,z), (x,y,z)
        std::unique_ptr<ExtendedKalman<double,3,3>> linear_filter_static;
        // (x,v_x,y,v_y,z,v_z), (pitch,yaw,distance)
        std::unique_ptr<ExtendedKalman<double,6,3>> linear_filter_cv_sphere;

        void setTransitionMatrix();
        void setProcessNoise();
        void setMeasurementNoise(double);
        double update_time;

        void getDifferentChiSquardValue();
        
        Eigen::Vector3d filtered_position_ca;
        Eigen::Vector3d filtered_position_cv;
        Eigen::Vector3d filtered_position_static;
        Eigen::Vector3d filtered_position_cv_sphere;

        Eigen::Vector3d filtered_speed_ca;
        Eigen::Vector3d filtered_speed_cv;
        Eigen::Vector3d filtered_speed_cv_sphere;

        Eigen::Vector3d filtered_acc_ca;

        double chi_squard_ca;
        double chi_squard_cv;
        double chi_squard_static;

        int followed_index;
    };
}
