#include <thread>
#include <chrono>
#include <umt.hpp>
#include <tracker.hpp>
#include <TrackingGate2D.hpp>
#include <detector.hpp>
#include <SmartLog.hpp>

using namespace LY_UTILS;
#define IMG_WIDTH 1280
#define IMG_HEIGHT 1024

Eigen::Vector2d center_vector{IMG_WIDTH/2,IMG_HEIGHT/2};
#define ZONE_STATUS(x,y) x.at(static_cast<int>(y)).zone_status 
#define EXECUTE_MIN_TIME 2.0
#define SHOOT_FREQUENCY 5
#define RECORD_SLIDING_WINDOW_FLAG true

namespace TRACKER
{
    // void analyzer_run() 
    // {
    //     auto armor_comparator = std::make_unique<ArmorComparator>();
    //     auto buff_comparator = std::make_unique<BuffComparator>();

    //     umt::Subscriber<TwoDetections> two_analyze_sub("two_detections");
    //     umt::Subscriber<SingleDetection> single_analyze_sub("single_detection");

    //     while(true)
    //     {
    //         //TODO
    //     }

    // }

    //计算当前dps
    float dpsCalculate(std::array<uint8_t, 4>& robot_buff,std::array<uint8_t, 4>& robot_gun)
    {
        float one_shot_damage = 1 + robot_buff.at(3)/100;
        int able_heat = (robot_gun.at(1) - robot_gun.at(2))/10;
        int cool_down = (robot_buff.at(1)*robot_gun.at(0))/10;
        float able_damage = one_shot_damage*((able_heat+cool_down)>SHOOT_FREQUENCY?SHOOT_FREQUENCY:able_heat+cool_down);

        return able_damage;
    }

    int getPiority(std::shared_ptr<TrackingZone> zone)
    {
        int piority;
        switch (zone->zone_tag_id)
        {
        case 0:
            piority = 0;
            break;
        case 1:
            piority = 10;
            break;
        case 2:
            piority = 1;
            break;
        case 3:
            piority = 6;
            break;
        case 4:
            piority = 5; 
            break;
        case 5:
            piority = 5;
            break; 
        case 6:
            piority = 7;
            break;
        case 7:
            piority = 2;
            break;
        }

        switch (static_cast<int>(zone->suspect_move_status))
        {
        case 0:
            piority *=1.5;
            break;
        case 1:
            piority *=1;
            break;
        case 2:
            piority *=0.8;
            break;
        }
        return piority;
    }

    std::shared_ptr<TrackingZone> judgeBestZone(std::vector<std::shared_ptr<TrackingZone>> zones,const int& number_want,DRIVER::SerialReadData::RobotStatus& robot_status)        
    {                
        //先强制number_want判定
        // if(ZONE_STATUS(zones,number_want) == ZoneStatus::UNSTABLE_ZONE)
        // {
        //     //有不稳定跟踪域，但操作手期望优先级高，等待跟踪域变稳定
        //     return zones.at(number_want);
        // }
        // else if(ZONE_STATUS(zones,number_want) == ZoneStatus::STABLE_ZONE)
        // {
        //     //number_want有稳定跟踪域，ok 
        //     return zones.at(number_want);
        // }
        // else
        // {
        //     //number_want无跟踪域，用普通优先级判断
        //     return TrackingZone();
        // }

        //普通优先级判断
        //根据跟踪域离屏幕中心距离，选取最近邻和次近邻跟踪域两个zone进行优先级选择

        //删除ZoneStatus不为STABLE的zone
        zones.erase(std::remove_if(zones.begin(), zones.end(), [](const auto& zone) {return zone->zone_status != ZoneStatus::STABLE_ZONE;}), zones.end());

        std::vector<std::pair<double, std::shared_ptr<TrackingZone>>> pairs;
        for (const auto& zone : zones) {
            double distance = (zone->zone_center - center_vector).norm();
            pairs.push_back({distance, zone});}

        //按距离升序排序
        std::sort(pairs.begin(), pairs.end(), [](const auto& a, const auto& b) {return a.first < b.first;});

        if(/*robot_status.enemy_health.at(6)>1*/false)//对面前哨站还在
        {
            //删掉哨兵
            pairs.erase(std::remove_if(pairs.begin(),pairs.end(),[](const auto& pair){return pair.second->zone_tag_id == 6;}),pairs.end());
        }
        if(pairs.size()>1)
        {
            if((pairs.at(1).first/pairs.at(0).first)>1.5)//距离相差过远
            {        
                return pairs.at(0).second;
            }
            else//两个跟踪域距离差不多远
            {
                // //斩杀线判断
                // int zone_one_enemy_health = robot_status.enemy_health.at(pairs.at(0).second.zone_tag_id);
                // int zone_two_enemy_health = robot_status.enemy_health.at(pairs.at(1).second.zone_tag_id);

                // float dps_now = dpsCalculate(robot_status.robot_buff,robot_status.robot_gun);

                // float hit_rate_one;
                // switch (pairs.at(0).second.suspect_move_status)
                // {
                // case MoveStatusSuspect::RELATIVE_STAND:
                //     hit_rate_one = 0.8;
                //     break;
                // case MoveStatusSuspect::RELATIVE_TRANSLATE:
                //     hit_rate_one = 0.6;
                //     break;
                // case MoveStatusSuspect::CLOCKWISE_SPIN:
                //     hit_rate_one = 0.5;
                //     break;
                // case MoveStatusSuspect::COUNTER_CLOCKWISE_SPIN:
                //     hit_rate_one = 0.5;
                //     break;
                // case MoveStatusSuspect::TWISTING:
                //     hit_rate_one = 0.6;
                //     break;
                // }

                // float hit_rate_two;
                // switch (pairs.at(1).second.suspect_move_status)
                // {
                // case MoveStatusSuspect::RELATIVE_STAND:
                //     hit_rate_two = 0.8;
                //     break;
                // case MoveStatusSuspect::RELATIVE_TRANSLATE:
                //     hit_rate_two = 0.6;
                //     break;
                // case MoveStatusSuspect::CLOCKWISE_SPIN:
                //     hit_rate_two = 0.5;
                //     break;
                // case MoveStatusSuspect::COUNTER_CLOCKWISE_SPIN:
                //     hit_rate_two = 0.5;
                //     break;
                // case MoveStatusSuspect::TWISTING:
                //     hit_rate_one = 0.6;
                //     break;
                // }
                
                // bool zone_one_in_execution = false ,zone_two_in_execution = false;
                // if(zone_one_enemy_health<EXECUTE_MIN_TIME*dps_now*hit_rate_one)
                // {
                //     zone_one_in_execution = true;
                // }
                // if(zone_two_enemy_health<EXECUTE_MIN_TIME*dps_now*hit_rate_two)
                // {
                //     zone_two_in_execution = true;
                // }

                // if(zone_one_in_execution&&zone_two_in_execution)
                // {
                //     return getPiority(pairs.at(0).second)>getPiority(pairs.at(1).second)?pairs.at(0).second:pairs.at(1).second;
                // }
                // else if(zone_one_in_execution)
                // {
                //     return pairs.at(0).second;
                // }
                // else if(zone_two_in_execution)
                // {
                //     return pairs.at(1).second;
                // }
                // else
                // {
                    return getPiority(pairs.at(0).second)>getPiority(pairs.at(1).second)?pairs.at(0).second:pairs.at(1).second;
                //}
            }
        }
        else
        {
            return pairs.at(0).second;
        }
    }

    void tracker_run() 
    {
        std::unique_ptr<TargetsTracker> targets_tracker = std::make_unique<TargetsTracker>();

        auto robot_status = umt::ObjManager<DRIVER::SerialReadData::RobotStatus>::find_or_create("robot_status");
        umt::Subscriber<DetectionPackage> detection_sub("detection_pack");

        umt::Publisher<TrackingPackage> tracking_pub("tracking_pack");
        int last_frame_aim_request = 0;
        int last_tracking_Zone_id = 0 ;

        while(true)
        {
            try{
                auto detection_pack = detection_sub.pop_for(200);
                //规定时限内获取到识别结果
                COUT("get detection package",GREEN);
                targets_tracker->inputNewMeasurements(detection_pack.armor_detection,detection_pack.time_stamp);
                bool record_flag = (detection_pack.imu_flag.number_want == 255) && RECORD_SLIDING_WINDOW_FLAG; 
                targets_tracker->setRecordFlag(record_flag);

                auto all_zones = targets_tracker->getTrackingZones(); 
                auto all_stable_zones = targets_tracker->getStableTrackingZones();

                if(all_stable_zones.empty())
                {
                    COUT("STABLE ZONE EMPTY", YELLOW);
                    continue;
                }
                
                std::shared_ptr<TrackingZone> best_zone_to_attack;
                    
                if(detection_pack.imu_flag.aim_request == 0)
                {
                    last_frame_aim_request = 0;
                    best_zone_to_attack = judgeBestZone(all_stable_zones,detection_pack.imu_flag.number_want,*robot_status.get());
                    last_tracking_Zone_id = -1;
                }
                else if(detection_pack.imu_flag.aim_request == 1 && last_frame_aim_request == 0)
                {
                    last_frame_aim_request = 1;
                    best_zone_to_attack = judgeBestZone(all_stable_zones,detection_pack.imu_flag.number_want,*robot_status.get());
                    last_tracking_Zone_id = best_zone_to_attack->zone_tag_id;
                }
                else if(detection_pack.imu_flag.aim_request == 1 && last_frame_aim_request == 1)
                {
                    //先判断上一次的tracking zone是否存活，优先锁上一次的tracking zone
                    //若无，则根据正常优先级选择方式去判断
                    last_frame_aim_request = 1;
                    if(all_zones.at(last_tracking_Zone_id)->zone_status == ZoneStatus::STABLE_ZONE || all_zones.at(last_tracking_Zone_id)->zone_status == ZoneStatus::UNSTABLE_ZONE)
                    {
                        best_zone_to_attack = all_zones.at(last_tracking_Zone_id);
                    }
                    else
                    {
                        best_zone_to_attack = judgeBestZone(all_stable_zones,detection_pack.imu_flag.number_want,*robot_status.get());
                    }
                    last_tracking_Zone_id = best_zone_to_attack->zone_tag_id;
                }
                
                COUT("best enemy to attack: "<<best_zone_to_attack->zone_tag_id,GREEN);

                std::vector<std::pair<int, BBox>> bboxes_with_index;
                if(best_zone_to_attack->gates_in_zone.empty())
                {
                    continue;
                }
                for (const auto& gate_ptr : best_zone_to_attack->gates_in_zone) {
                    const TrackingGate2D& gate = *gate_ptr;
                    bboxes_with_index.push_back(std::make_pair(gate.armor_index, gate.latest_bbox_inside));
                }
                tracking_pub.push({bboxes_with_index,best_zone_to_attack->suspect_move_status,detection_pack.imu_flag,detection_pack.time_stamp});

            } catch (umt::MessageError_Timeout &e) {
                //规定时限内未获取识别结果
                targets_tracker->remindAllTrackersTimeOut();
                COUT("[WARNING] 'detection_pack' "<<e.what(),RED);
            } catch (umt::MessageError &e) {
                COUT("[WARNING] 'detection_pack' "<<e.what(),RED);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
    }

    void bkg_tracker_run(){
        std::thread([=](){
            tracker_run();
        }).detach();
    }

PYBIND11_EMBEDDED_MODULE(TRACKER_, m) {
namespace py = pybind11;
m.def("bkg_tracker_run", bkg_tracker_run);
}

} // namespace TRACKER
