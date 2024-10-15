#include "AngleTimeSolver.hpp"
#include "umt.hpp"

#define BIG_ARMOR_LENGTH 0.1125f
#define SMALL_ARMOR_LENGTH 0.0675f
#define ARMOR_HEIGHT 0.0275f

#include <numeric> 

namespace PREDICTOR
{
    float AngleTimeSolver::getBulletFlyTime(Eigen::Vector3d& point)
    {        
        float distance = sqrt(point[0] * point[0] + point[2] * point[2]);
        float pitch = atan2(point[1], distance);
        float time = abs(distance/this->average_speed*cos(pitch));
        return time;
    }

    YawAndDistanceBound AngleTimeSolver::getGoalArea(CenterAndRange& aim_center_and_range, SpinState spin_state)
    {
        Eigen::Vector3d aim_center = aim_center_and_range.aim_center;
        double aim_range = aim_center_and_range.aim_range;
        // std::cout<<"center"<<aim_center<<std::endl;
        YawAndPitch yaw_pitch_center = calculateYawPitch(aim_center);
        double yaw_range = asin(aim_range / std::sqrt(aim_center[0] * aim_center[0] + aim_center[1] + aim_center[1])) * 180 / M_PI;
        std::cout<<"yaw_range:"<<yaw_range<<std::endl;

        double distance = std::sqrt(aim_center[0] * aim_center[0] + aim_center[1] * aim_center[1]);

        double upper_yaw, lowwer_yaw;
        if(spin_state == SpinState::CLOCK_WISE_STATE)
        {
            upper_yaw = yaw_pitch_center.first + 1.0 * yaw_range;
            lowwer_yaw = yaw_pitch_center.first - 1.0 * yaw_range;
        }
        else if(spin_state == SpinState::COUNTER_CLOCK_WISE_STATE)
        {
            upper_yaw = yaw_pitch_center.first + 1.0 * yaw_range;
            lowwer_yaw = yaw_pitch_center.first - 1.0 * yaw_range;
        }
        else
        {
            upper_yaw = yaw_pitch_center.first + yaw_range;
            lowwer_yaw = yaw_pitch_center.first - yaw_range;
        }

        //std::cout<<"distance bound"<<distance<<std::endl;

        return {yaw_pitch_center.first, lowwer_yaw, upper_yaw, distance};
    }

    std::pair<YawAndPitchRegion,Distance> AngleTimeSolver::getYawPitchRegion(std::vector<Eigen::Vector3d>& points_after_fly, LY_UTILS::ARMOR_SIZE& armor_size, YawAndDistanceBound yaw_and_distance_bound, FollowMode follow_mode, SpinState spin_state)
    {
        umt::Publisher<Eigen::Vector3d> point_hit_pub("point_hit");

        YawAndPitchRegion region_temp;
        std::vector<double> distance_vec;
        std::vector<Eigen::Vector3d> point_temp_vec;

        for(auto point : points_after_fly)
        {
            // // 剔除掉距离大于上界的点
            // if(std::sqrt(point[0] * point[0] + point[1] * point[1]) > yaw_and_distance_bound.distance_upper_bound && follow_mode != FollowMode::LINEAR_TRACKING)
            // {
		    //     std::cout<<"distance delete"<<std::endl;
            //     continue;
            // }
            YawAndPitch yaw_pitch_center = calculateYawPitch(point);
            // if((yaw_pitch_center.first > yaw_and_distance_bound.yaw_upper_bound || yaw_pitch_center.first < yaw_and_distance_bound.yaw_lower_bound) && follow_mode != FollowMode::LINEAR_TRACKING)
            // {
            //     std::cout<<"yaw:"<<yaw_pitch_center.first<<" yaw_up:"<<yaw_and_distance_bound.yaw_upper_bound<<" yaw_low:"<<yaw_and_distance_bound.yaw_lower_bound<<std::endl;
            //     std::cout<<"yaw region delete"<<std::endl;
            //     continue;
            // }
            point_temp_vec.push_back(point);

            YawAndPitchThresh yaw_pitch_thresh;
            if(armor_size == LY_UTILS::ARMOR_SIZE::BIG_ARMOR)
            { 
                yaw_pitch_thresh = {atan2(BIG_ARMOR_LENGTH, point.norm()), atan2(ARMOR_HEIGHT, point.norm())};
            }
            else
            {
                yaw_pitch_thresh = {atan2(SMALL_ARMOR_LENGTH, point.norm()), atan2(ARMOR_HEIGHT, point.norm())};
            }
            distance_vec.push_back(std::sqrt(point[0] * point[0] + point[1] * point[1]));
            region_temp.push_back({yaw_pitch_center,yaw_pitch_thresh});
        }

        // 对 distance_vec 进行排序，并根据排序结果对 region_temp 进行排序
        std::vector<size_t> indices(distance_vec.size());
        std::iota(indices.begin(), indices.end(), 0); // 生成索引序列

        std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
            return distance_vec[a] < distance_vec[b];
        });

        YawAndPitchRegion sorted_region_temp;
        for (size_t i : indices) {
            sorted_region_temp.push_back(region_temp[i]);
        }

        std::vector<Eigen::Vector3d> sorted_point_temp_vec;
        for (size_t i : indices) {
            sorted_point_temp_vec.push_back(point_temp_vec[i]);
        }

        if(follow_mode != FollowMode::LINEAR_TRACKING)
        {
            sorted_region_temp.pop_back();
            sorted_region_temp.pop_back();
            distance_vec.pop_back();
            distance_vec.pop_back();
            if(spin_state == SpinState::CLOCK_WISE_STATE)
            {
                if(sorted_region_temp.at(0).first.first < sorted_region_temp.at(1).first.first)
                {
                    return std::make_pair(sorted_region_temp, distance_vec);
                }
                else
                {
                    std::swap(sorted_region_temp[0], sorted_region_temp[1]);
                    std::swap(distance_vec[0], distance_vec[1]);
                    return std::make_pair(sorted_region_temp, distance_vec);
                }
            }
            else if (spin_state == SpinState::COUNTER_CLOCK_WISE_STATE)
            {
                if(sorted_region_temp.at(0).first.first < sorted_region_temp.at(1).first.first)
                {
                    std::swap(sorted_region_temp[0], sorted_region_temp[1]);
                    std::swap(distance_vec[0], distance_vec[1]);
                    return std::make_pair(sorted_region_temp, distance_vec);
                }
                else
                {  
                    return std::make_pair(sorted_region_temp, distance_vec);
                }
            }
            
        }

        // point_hit_pub.push(sorted_point_temp_vec.at(0));
        // std::cout<<"point:"<<sorted_point_temp_vec.at(0)<<std::endl;

        return std::make_pair(sorted_region_temp, distance_vec);
    }

    YawAndPitch AngleTimeSolver::calculateYawPitch(Eigen::Vector3d& point)
    {
        float yaw = -atan2(point[0], point[1]);
        float distance = sqrt(point[0] * point[0] + point[1] * point[1]);
        float pitch = atan2(point[2], distance);

        float y_temp = point[2];

        float dy, a, y_actual;
        for (int i = 0; i < 20; i++)
        {
            a = (float)atan2(y_temp, distance);
            y_actual = bulletModel(distance, a);
            dy = point[2] - y_actual;
            y_temp = y_temp + dy;
            if (fabs(dy) < 0.001)
            {
                break;
            }
        }
        float pitch_final = (float)atan2(y_temp, distance);

        return std::make_pair(yaw / 3.1415926 * 180.0, pitch_final / 3.1415926 * 180.0);
    }

    float AngleTimeSolver::bulletModel(float x, float angle)
    {
        float k_wind = 0.001;
        float t,y;
        float v = this->average_speed;
        t = (float)((exp(k_wind * x) - 1) / (k_wind * v * cos(angle)));
        y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
        return y;
    }

    void AngleTimeSolver::calculateAverageBulletSpeed(float actual_bullet_speed)
    {
        speed_window.push(actual_bullet_speed);
        total_speed += actual_bullet_speed;
        if(speed_window.size()>WINDOW_SIZE){
            total_speed -= speed_window.front();
            speed_window.pop();
        }
        if(speed_window.size()>0){
            this->average_speed = total_speed / speed_window.size();
        } else {
            this->average_speed = DEFAULT_BULLET_SPEED;
        }
    }
}
