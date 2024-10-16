#pragma once

#include <vector>
#include "Eigen/Core"
#include "utils.h"
#include "AngleTimeSolver.hpp"

namespace PREDICTOR
{
    class FireController
    {
    public:
        YawAndPitch getAimYawPitch(float yaw_now, float pitch_now, std::pair<YawAndPitchRegion,Distance> yaw_pitch_region_with_distance, YawAndDistanceBound yaw_and_distance_bound);
        uint8_t getShootFlag(YawAndPitch,YawAndPitch, FollowMode follow_mode);
    private:
        float last_aim_yaw;
        float last_aim_pitch;
    };

    YawAndPitch FireController::getAimYawPitch(float yaw_now, float pitch_now, std::pair<YawAndPitchRegion,Distance> yaw_pitch_region_with_distance, YawAndDistanceBound yaw_and_distance_bound)
    {
        // std::cout<<"useful"<<yaw_pitch_region_with_distance.first.size()<<std::endl;
        return yaw_pitch_region_with_distance.first.at(0).first;
        // if(yaw_pitch_region_with_distance.first.size() > 1)
        // {
        //     if(yaw_pitch_region_with_distance.second.at(1) > 1.5 * yaw_pitch_region_with_distance.second.at(0))
        //     {
        //         std::cout<<"follow by distance"<<std::endl;
        //         return yaw_pitch_region_with_distance.first.at(0).first;
        //     }
        //     else
        //     {
        //         std::cout<<"follow by yaw"<<std::endl;
        //         if(std::abs(yaw_pitch_region_with_distance.first.at(1).first.first - yaw_now) < std::abs(yaw_pitch_region_with_distance.first.at(0).first.first - yaw_now))
        //         {
        //             return yaw_pitch_region_with_distance.first.at(1).first;
        //         }
        //         else
        //         {
        //             return yaw_pitch_region_with_distance.first.at(0).first;
        //         }
        //     }
        // }
        // else
        // {
        //     return yaw_pitch_region_with_distance.first.at(0).first;
        // }
    }
    
    uint8_t FireController::getShootFlag(YawAndPitch y_p_set_point, YawAndPitch y_p_now, FollowMode follow_mode)
    {
        // auto aim_offset = umt::ObjManager<std::pair<float, float>>::find_or_create("aim_offset");

        // auto *this_aim_offset = aim_offset.get();

        double yaw_diff, pitch_diff;
        if(follow_mode == FollowMode::LINEAR_TRACKING)
        {
            yaw_diff = 3.0;
            pitch_diff = 3.0;
        }
        else
        {
            yaw_diff = 3.0;
            pitch_diff = 3.0;
        }

        y_p_set_point.first = (float)(y_p_set_point.first + std::round((y_p_now.first - y_p_set_point.first) / 360.0) * 360.0);
        if(std::abs(y_p_set_point.first - y_p_now.first) < yaw_diff && std::abs(y_p_set_point.second - y_p_now.second) < pitch_diff)
        {
            return static_cast<uint8_t>(1);
        }
        else
        { 
            return static_cast<uint8_t>(0);
        }
    }
} //namespace PREDICTOR
