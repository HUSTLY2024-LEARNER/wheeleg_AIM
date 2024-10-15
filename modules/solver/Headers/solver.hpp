#pragma once
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "driver.hpp"
#include "sophus/se3.hpp"
#include "tracker.hpp"
#include "utils.h"
#include "detector.hpp"

using namespace LY_UTILS;

namespace SOLVER
{
    struct ArmorPose
    {
        Eigen::Vector3d translate;
        double yaw_world;
    };

    typedef std::vector<std::pair<int,ArmorPose>> IndexedArmorPoses;

    struct SolutionPackage
    {
        IndexedArmorPoses armor_poses;
        ENEMY_TYPE enemy_type;
        TRACKER::MoveStatusSuspect move_status;
        DRIVER::SerialReadData::IMU_Flag imu_flag;
        long time_stamp;
    };
} //namespace SOLVER