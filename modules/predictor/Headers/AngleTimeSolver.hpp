#pragma once

#include <queue>
#include "Eigen/Core"
#include "utils.h"

#define WINDOW_SIZE 20
#define DEFAULT_BULLET_SPEED 25

#define GRAVITY 9.8

namespace PREDICTOR
{
    
using YawAndPitch = std::pair<float,float>;
using YawAndPitchThresh = std::pair<float,float>;
using YawAndPitchRegion = std::vector<std::pair<YawAndPitch,YawAndPitchThresh>>;
using Distance = std::vector<double>;

    enum class FollowMode
    {
        LINEAR_TRACKING,
        DENSE_TRACKING,
        SPARSE_TARACKING,
        STAND_WAITING
    };
    
    enum class SpinState
    {
        UNKNOW_STATE,
        COUNTER_CLOCK_WISE_STATE,
        CLOCK_WISE_STATE
    };
    
    struct CenterAndRange
    {
        Eigen::Vector3d aim_center;
        double aim_range;
    };

    struct YawAndDistanceBound
    {
        double yaw_center;
        double yaw_lower_bound;
        double yaw_upper_bound;
        double distance_upper_bound;
    };

    class AngleTimeSolver
    {
    public:
        void calculateAverageBulletSpeed(float actual_bullet_speed);
        float getBulletFlyTime(Eigen::Vector3d& point);
        YawAndDistanceBound getGoalArea(CenterAndRange& aim_center_and_range, SpinState spin_state);
        std::pair<YawAndPitchRegion,Distance> getYawPitchRegion(std::vector<Eigen::Vector3d>& points_after_fly, LY_UTILS::ARMOR_SIZE& armor_size, YawAndDistanceBound yaw_and_distance_bound, FollowMode follow_mode, SpinState spin_state);
    private:
        std::queue<float> speed_window;
        float total_speed = 0.0;
        float average_speed = DEFAULT_BULLET_SPEED;
        YawAndPitch calculateYawPitch(Eigen::Vector3d& point);
        float bulletModel(float x, float angle);
    };

} // namespace PREDICTOR
