#include "TrackingGate2D.hpp"
#include "BBoxes.h"
#include "SmartLog.hpp"
#include <umt.hpp>
#include <fstream>

using namespace DETECTOR;
#define MAX_GATE_LOSS_TIME 50
#define MAX_GATE_LOSS_COUNT 2
#define MIN_GATE_MAINTAIN_COUNT 1
#define DIED_REMAIN_COUNT 50
#define GATES_EMPTY false
#define GATES_NOT_EMPTY true
#define VERIFY_THRESH 500
#define DEFAULT_DELTA_TIME 10
#define WINDOW_LENGTH 100

using namespace LY_UTILS;
namespace TRACKER
{
    ///////////////TargetsTracker///////////////
    TargetsTracker::TargetsTracker()
    {
        for(int i =0;i<8;i++)
        {
            this->trackers.push_back(SingleTypeTracker(i));
        }
    }

    void TargetsTracker::inputNewMeasurements(const BBoxes& bboxes,long &time_stamp)
    {
        for(auto& tracker :this->trackers)
        {
            tracker.updateTimeStamp(time_stamp);
        }

        BBoxes single_type_bboxes[8];
        for (int i = 0; i < 8; i++)
        {
            single_type_bboxes[i] = BBoxes(); // 初始化每个元素为一个空的 BBoxes 对象
        }
        for(const auto& bbox : bboxes)
        {
            single_type_bboxes[bbox.tag_id].push_back(bbox);
        }
        for(int i = 0; i < 8 ; i++)
        {
            if(!single_type_bboxes[i].empty())
            {
                trackers.at(i).inputSingleTypeMeasurements(single_type_bboxes[i],time_stamp);
            }
        }
        for(auto& tracker :this->trackers)
        {
            tracker.mainTainTrackingGates(false);
        }
        COUT("measurements input success",GREEN);
    }

    void TargetsTracker::remindAllTrackersTimeOut()
    {
        COUT("REMIND TIME OUT",RED);
        for(auto& tracker :this->trackers)
        {
            tracker.mainTainTrackingGates(true);
        }
    }

    const std::vector<std::shared_ptr<TrackingZone>> TargetsTracker::getTrackingZones() const
    {
        std::vector<std::shared_ptr<TrackingZone>> zones_temp;
        for(auto& tracker :this->trackers)
        {
            zones_temp.push_back(tracker.getTrackingZone());
        }
        return zones_temp;
    }

    const std::vector<std::shared_ptr<TrackingZone>> TargetsTracker::getStableTrackingZones() const
    {
        std::vector<std::shared_ptr<TrackingZone>> zones_temp;
        for(auto& tracker :this->trackers)
        {
            if(tracker.getTrackingZone().get()->zone_status == ZoneStatus::STABLE_ZONE)
            {
                zones_temp.push_back(tracker.getTrackingZone());
            }
        }
        return zones_temp;
    }

    void TargetsTracker::setRecordFlag(const bool& flag)
    {
        for(auto& tracker :this->trackers)
        {
            tracker.setRecordFlag(flag);
        }
    }

    ///////////////SingleTypeTracker///////////////
    void SingleTypeTracker::updateTimeStamp(const long& time_stamp)
    {
        this->time_stamp = time_stamp;
    }

    void SingleTypeTracker::inputSingleTypeMeasurements(const BBoxes& bboxes, long& time_stamp)
    {   
        this->single_type_gates_copy = this->single_type_gates;
        std::vector<int> erased_gate_ids;

        for (int i = 0; i < bboxes.size(); i++)
        {
            int gate_id = findClosestGateID(bboxes.at(i), i);

            if (gate_id == -1) // 当前单类别tracker下面没有和观测量匹配的gate，直接创建一个新的gate
            {
                createNewTrackingGate(bboxes.at(i));
            }
            else
            {
                bool in_gate_or_not = this->single_type_gates_copy.at(gate_id)->judgeNewBBox(bboxes.at(i));
                if (!in_gate_or_not) {
                    createNewTrackingGate(bboxes.at(i));
                }
                else {
                    int count = std::count_if(erased_gate_ids.begin(), erased_gate_ids.end(), [gate_id](int num){
                        return num <= gate_id;
                    });

                    if(gate_id + count <this->single_type_gates.size())
                    {
                        this->single_type_gates.at(gate_id + count)->inputNewBBox_thenUpdate(bboxes.at(i), time_stamp);
                    }

                    this->single_type_gates_copy.erase(single_type_gates_copy.begin() + gate_id);
                    erased_gate_ids.push_back(gate_id);
                }
            }
        }

        // updateLossGatesPosition();
    }

    void SingleTypeTracker::mainTainTrackingGates(const bool &detection_loss)
    {
        if(died_numbered_gates.size() > 1)
        {
            died_numbered_gates.pop_front();
        }

        for(auto &died_gate : this->died_numbered_gates)
        {
            died_gate->gate_loss_count ++;
        }
        this->died_numbered_gates.erase(std::remove_if(died_numbered_gates.begin(), died_numbered_gates.end(), 
            [](const std::shared_ptr<TrackingGate2D>& gate_ptr) {
            return gate_ptr->gate_loss_count > DIED_REMAIN_COUNT;
            }), died_numbered_gates.end());

        //每次有新观测时，判断每个跟踪门的status
        for(auto &gate : this->single_type_gates)
        {  
            //时间上丢失超时或者此周期内无识别结果
            if(this->time_stamp - gate->latest_time_stamp_to_be_updated >MAX_GATE_LOSS_TIME || detection_loss)
            {
                COUT("TRACKER "<<this->single_tag_id<<" LOSS________________",YELLOW);
                 gate->position = gate->gate_kf->getPredictedPosition();
                if(gate->gate_status == GateStatus::STABLE_GATE)
                {
                    gate->gate_loss_count++;
                    gate->gate_status = GateStatus::UNSTABLE_LOSS_GATE;
                }
                else if(gate->gate_status == GateStatus::UNSTABLE_BUMP_GATE)
                {
                    gate->gate_maintain_count = 0;
                    gate->gate_loss_count++;
                    gate->gate_status = GateStatus::DIED_GATE;
                }
                else if(gate->gate_status == GateStatus::UNSTABLE_LOSS_GATE)
                {
                    gate->gate_loss_count++;
                    if(gate->gate_loss_count<MAX_GATE_LOSS_COUNT){
                        gate->gate_status = GateStatus::UNSTABLE_LOSS_GATE;
                    }else {
                        gate->gate_status = GateStatus::DIED_GATE;
                    }
                }
            }
            else//时间上未丢失
            {
                COUT("TRACKER "<<this->single_tag_id<<" BUMP________________",BLUE);
                if(gate->gate_status == GateStatus::STABLE_GATE)
                {
                    gate->gate_status = GateStatus::STABLE_GATE;
                }
                else if(gate->gate_status == GateStatus::UNSTABLE_BUMP_GATE)
                {
                    gate->gate_maintain_count++;
                    gate->gate_loss_count = 0;
                    if(gate->gate_maintain_count>MIN_GATE_MAINTAIN_COUNT){
                        gate->gate_status = GateStatus::STABLE_GATE;
                    }else {
                        gate->gate_status = GateStatus::UNSTABLE_BUMP_GATE;
                    }
                }
                else if(gate->gate_status == GateStatus::UNSTABLE_LOSS_GATE)
                {
                    gate->gate_loss_count = 0;
                    gate->gate_status = GateStatus::STABLE_GATE;
                }
            }
        }

        //如果gate.gate_status为DIED_GATE，清除这个gate
        std::copy_if(single_type_gates.begin(), single_type_gates.end(), std::back_inserter(this->died_numbered_gates),
            [](const std::shared_ptr<TrackingGate2D>& gate_ptr) {
            return gate_ptr->gate_status == GateStatus::DIED_GATE && gate_ptr->is_indexed == true; 
            });
        
        this->single_type_gates.erase(std::remove_if(single_type_gates.begin(), single_type_gates.end(), 
            [](const std::shared_ptr<TrackingGate2D>& gate_ptr) {
            return gate_ptr->gate_status == GateStatus::DIED_GATE;
            }), single_type_gates.end());

        if(!this->single_type_gates.empty()){
            COUT("TRACKER "<<this->single_tag_id<<" HAVE "<<this->single_type_gates.size()<<" GATES",GREEN); 
            //给所有门编号之后维护trackingzone
            numberingAllGates();
            maintainTrackingZone(GATES_NOT_EMPTY);
        }
        else{
            // COUT("TRACKER "<<this->single_tag_id<<" GATES EMPTY",YELLOW);
            maintainTrackingZone(GATES_EMPTY);
        }
    }

    //给所有门编号，仅考虑stable和loss的gate（bump的不参与，防止一帧误识别就造成后续排序的错乱）
    //同一时刻视野内仅可能有两块装甲板，考虑到可能有一块装甲板刚好转走，此时情况有以下几种：
    //0.无gate
    //1.一个正面的gate为stable，没有其余gate
    //2.一个正面的gate为stable，一个刚好转走的为loss
    //3.一个正面的gate为loss(识别掉帧，或被击打灯条熄灭)，没有其余gate
    //4.一个正面的gate为loss(识别掉帧，或被击打灯条熄灭)，一个刚好转走的为loss
    //5.两个正面的gate为stable，没有其余gate
    //6.两个正面的gate为stable，一个刚好转走的为loss
    //7.两个正面的gate一个为stable，一个为loss(识别掉帧，或被击打灯条熄灭)，没有其余gate
    //8.两个正面的gate一个为stable，一个为loss(识别掉帧，或被击打灯条熄灭)，一个刚好转走的为loss
    bool SingleTypeTracker::numberingAllGates()
    {
        umt::Publisher<std::vector<std::vector<double>>> track_pub("track_gate");

        std::sort(this->single_type_gates.begin(), this->single_type_gates.end(),
            [](const std::shared_ptr<TrackingGate2D>& gate1, const std::shared_ptr<TrackingGate2D>& gate2) {
                if (gate1->position[0] == gate2->position[0]) {
                    return gate1->position[1] < gate2->position[1];
                }
                return gate1->position[0] < gate2->position[0];
            });

        if (!gate_base_exist) // 不存在gate_base
        {
            int stable_gate_numbers = std::count_if(this->single_type_gates.begin(), this->single_type_gates.end(),
                [](const std::shared_ptr<TrackingGate2D>& gate) { return gate->gate_status == GateStatus::STABLE_GATE; });

            if (stable_gate_numbers == 0) {
                return false;
            } else if (stable_gate_numbers == 1) {
                auto it = std::find_if(this->single_type_gates.begin(), this->single_type_gates.end(),
                    [](const std::shared_ptr<TrackingGate2D>& gate) {
                        return gate->gate_status == GateStatus::STABLE_GATE;
                    });
                (*it)->armor_index = 0; 
                (*it)->is_indexed = true;   
                this->gate_base_exist = true;
                return true;
            } else if (stable_gate_numbers == 2) {
                auto it1 = std::find_if(this->single_type_gates.begin(), this->single_type_gates.end(),
                    [](const std::shared_ptr<TrackingGate2D>& gate) {
                        return gate->gate_status == GateStatus::STABLE_GATE;
                    });
                auto it2 = std::find_if(std::next(it1), this->single_type_gates.end(),
                    [](const std::shared_ptr<TrackingGate2D>& gate) {
                        return gate->gate_status == GateStatus::STABLE_GATE;
                    });
                (*it1)->armor_index = 0;
                (*it1)->is_indexed = true; 
                (*it2)->armor_index = 1;
                (*it2)->is_indexed = true;
                this->gate_base_exist = true;
                return true;
            } else {
                return false;
            }
        }
        else // 已经存在gate_base，已经经过初始化，后续维护
        {
            std::vector<std::vector<double>> track_gate_temp;

            COUT("GATE BASE EXIST", CYAN);
            auto it = std::find_if(this->single_type_gates.begin(), this->single_type_gates.end(),
                [](const std::shared_ptr<TrackingGate2D>& gate) {
                    return gate->gate_status == GateStatus::STABLE_GATE && !gate->is_indexed;
                });
            
            if (it == this->single_type_gates.end()) {
                std::vector<std::pair<Eigen::Vector2d,int>> indexed_true_gates_position_and_index;
                for(auto& gate: this->single_type_gates)
                    {

                        int aaaa = 0;
                        if(gate->is_indexed)
                        {
                            indexed_true_gates_position_and_index.push_back(std::make_pair(gate->position,gate->armor_index));

                            if(gate->gate_status == GateStatus::STABLE_GATE)
                            {
                                aaaa = 1;
                            }

                            // std::cout<<"index: "<<gate->armor_index<<std::endl;
                        }
                        track_gate_temp.push_back({gate->position[0],gate->position[1],(double)gate->armor_index,(double)aaaa,gate->area});
                    }
                track_pub.push(track_gate_temp);
                return true;
            } else {
                if (ifGateInZone(**it)) {
                    std::vector<std::pair<Eigen::Vector2d,int>> indexed_true_gates_position_and_index;
                    for(auto& gate: this->single_type_gates)
                    {
                        if(gate->is_indexed)
                        {
                            indexed_true_gates_position_and_index.push_back(std::make_pair(gate->position,gate->armor_index));
                            // std::cout<<"index: "<<gate->armor_index<<std::endl;
                        }
                    }

                    if(indexed_true_gates_position_and_index.empty())
                    {
                        for(auto &died_gate : this->died_numbered_gates)
                        {
                            indexed_true_gates_position_and_index.push_back(std::make_pair(died_gate->position,died_gate->armor_index));
                            // std::cout<<"died index: "<<died_gate->armor_index<<std::endl;
                        }
                    }

                    std::sort(indexed_true_gates_position_and_index.begin(), indexed_true_gates_position_and_index.end(),
                        [](const auto &p_0, const auto &p_1){
                            return p_0.first[0] < p_1.first[0];
                        });

                    if ((*it)->position[0] < indexed_true_gates_position_and_index.front().first[0]) {
                        COUT("////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////",RED);
                        (*it)->armor_index = (indexed_true_gates_position_and_index.front().second + 4 - 1) % 4;
                        (*it)->is_indexed = true;
                        return true;
                    }
                    else if ((*it)->position[0] > indexed_true_gates_position_and_index.back().first[0]) {
                        COUT("////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////",BLUE);
                        (*it)->armor_index = (indexed_true_gates_position_and_index.back().second + 1) % 4;
                        (*it)->is_indexed = true;
                        return true;
                    }
                    else {
                        COUT("////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////",YELLOW);
                        return false;
                    }
                }
                else {
                    this->single_type_zone->setConfusingFactor(**it);
                    it = this->single_type_gates.erase(it);
                    return false;
                }
            }
        }
    }

    bool SingleTypeTracker::ifGateInZone(const TrackingGate2D& gate)
    {
        return true;
    }

    //根据tracker类内gates维护zone
    void SingleTypeTracker::maintainTrackingZone(const bool& empty_flag) {

        if (!empty_flag) {
            this->gate_base_exist = false;
            this->single_type_zone->destroyItself();
        } else {
            if (!this->single_type_zone->initializeState()) { // 没有初始化过 single_type_zone
                // 如果有 stable gate，进行初始化
                std::vector<std::shared_ptr<TrackingGate2D>> stable_gates;
                std::copy_if(this->single_type_gates.begin(), this->single_type_gates.end(),
                    std::back_inserter(stable_gates),
                    [](const std::shared_ptr<TrackingGate2D>& gate_ptr) {
                        return gate_ptr->gate_status == GateStatus::STABLE_GATE;
                    });

                if (!stable_gates.empty()) {
                    this->single_type_zone = std::make_shared<TrackingZone>(stable_gates);
                }
            }
            else // 已经初始化过，则进行维护
            {
                // 1. gates_with_bboxes——zone中的gates
                std::vector<std::shared_ptr<TrackingGate2D>> stable_gates;
                std::copy_if(this->single_type_gates.begin(), this->single_type_gates.end(),
                    std::back_inserter(stable_gates),
                    [](const std::shared_ptr<TrackingGate2D>& gate_ptr) {
                        return gate_ptr->gate_status == GateStatus::STABLE_GATE;
                    });

                std::vector<std::shared_ptr<TrackingGate2D>> loss_gates;
                std::copy_if(this->single_type_gates.begin(), this->single_type_gates.end(),
                    std::back_inserter(loss_gates),
                    [](const std::shared_ptr<TrackingGate2D>& gate_ptr) {
                        return gate_ptr->gate_status == GateStatus::UNSTABLE_LOSS_GATE;
                    });

                this->single_type_zone->gates_in_zone.clear();
                COUT("STABLE GATE SIZE"<<stable_gates.size(), RED);
                if(stable_gates.size()>1)
                {
                    if(stable_gates.at(1).get()->latest_bbox_inside.area > 1.2 * stable_gates.at(0).get()->latest_bbox_inside.area)
                    {
                        this->single_type_zone->gates_in_zone.push_back(std::make_shared<TrackingGate2D>(*stable_gates.at(1)));
                        this->single_type_zone->gates_in_zone.push_back(std::make_shared<TrackingGate2D>(*stable_gates.at(0)));
                    }
                    else
                    {
                        this->single_type_zone->gates_in_zone.push_back(std::make_shared<TrackingGate2D>(*stable_gates.at(0)));
                        this->single_type_zone->gates_in_zone.push_back(std::make_shared<TrackingGate2D>(*stable_gates.at(1)));
                    }
                }
                else if(stable_gates.size()>0)
                {
                    this->single_type_zone->gates_in_zone.push_back(std::make_shared<TrackingGate2D>(*stable_gates.at(0)));
                }

                // 2. zone_status
                int stable_gate_numbers = stable_gates.size();
                int loss_gate_numbers = loss_gates.size();

                if (stable_gate_numbers > 0) {
                    this->single_type_zone->zone_status = ZoneStatus::STABLE_ZONE;
                } else if (stable_gate_numbers == 0 && loss_gate_numbers > 0) {
                    this->single_type_zone->zone_status = ZoneStatus::UNSTABLE_ZONE;
                } else {
                    this->single_type_zone->destroyItself();
                }

                // 3. zone_center
                std::vector<std::shared_ptr<TrackingGate2D>> combined_gates;
                combined_gates.reserve(stable_gates.size() + loss_gates.size());
                std::copy(stable_gates.begin(), stable_gates.end(), std::back_inserter(combined_gates));
                std::copy(loss_gates.begin(), loss_gates.end(), std::back_inserter(combined_gates));

                Eigen::Vector2d zone_center_temp;
                float area_sum = 0;
                cv::Point2f center_cv_temp(0, 0);
                for (const auto& gate : combined_gates) {
                    area_sum += gate->latest_bbox_inside.area;
                    center_cv_temp += gate->latest_bbox_inside.area * gate->latest_bbox_inside.center;
                }
                center_cv_temp /= area_sum;
                zone_center_temp[0] = center_cv_temp.x;
                zone_center_temp[1] = center_cv_temp.y;
                this->single_type_zone->zone_center = zone_center_temp;

                // 4. suspect_move_status
                MoveStatusSuspect move_status = this->single_type_zone->getMoveStatusFromSlidingWindow(combined_gates);
                this->single_type_zone->suspect_move_status = move_status;
            }
        }
    }

    void SingleTypeTracker::createNewTrackingGate(const BBox& bbox)
    {
        std::shared_ptr<TrackingGate2D> gate_to_be_initialized = std::make_shared<TrackingGate2D>(bbox, this->time_stamp);
        gate_to_be_initialized->calculateDeltaT(this->time_stamp);
        gate_to_be_initialized->gate_kf->updateDeltaT(gate_to_be_initialized->last_delta_t);
        this->single_type_gates.push_back(std::move(gate_to_be_initialized));
    }

    int SingleTypeTracker::findClosestGateID(const BBox& bbox, const int& call_times)
    {
        double min_distance = 99999999.0;
        int gate_id = -1;
        for (auto gateIt = single_type_gates_copy.begin(); gateIt != single_type_gates_copy.end(); gateIt++)
        {
            Eigen::Vector2d predicted_position = (*gateIt)->predictThen_getPredictedPosition(call_times);
            double distance = (predicted_position - Eigen::Vector2d(bbox.center.x, bbox.center.y)).norm();
            if (distance < min_distance)
            {
                min_distance = distance;
                gate_id = std::distance(single_type_gates_copy.begin(), gateIt);
            }
        }
        return gate_id;
    }

    ///////////////TrackingZone///////////////
    TrackingZone::TrackingZone(std::vector<std::shared_ptr<TrackingGate2D>>& gates) : initialized(true), call_count(0), save_count(0), record_flag(false), gates_in_zone(std::move(gates)), zone_center(Eigen::Vector2d::Zero())
    {

        COUT("NEW ZONE|||||||||||||||||||||||||||",RED);
        this->suspect_move_status = MoveStatusSuspect::UNKNOW;

        float area_sum = 0.f;
        cv::Point2f center_cv_temp(0, 0);
        for (const auto& gate_ptr : gates_in_zone) {
            area_sum += gate_ptr->latest_bbox_inside.area;
            center_cv_temp += gate_ptr->latest_bbox_inside.area * gate_ptr->latest_bbox_inside.center;
        }
        center_cv_temp /= area_sum;
        this->zone_center[0] = center_cv_temp.x;
        this->zone_center[1] = center_cv_temp.y;
        this->zone_status = ZoneStatus::UNSTABLE_ZONE;
        this->zone_tag_id = gates_in_zone.at(0)->tag_id;
    }


    //先利用armor_index的切换来判断是否属于clockwise_spin/counter_clockwise_spin/twisting/else...
    //建立一个时间滑窗，内部存放了N帧内所有gate的armor_index情况
    //方案一：通过传统办法计算滑窗内装甲板从左边和从右边涌现或消失的次数，进而判断是否在顺时针小陀螺/逆时针小陀螺/扭腰（or无旋转）
    //缺点：规则难设计、需要有很多上下限阈值需要调参、统计学方法很难考虑时序连贯性、很难做到对滑窗未来一段时间的预测
    //方案二：通过机器学习/深度学习方法，将问题变为对一个4*N的图像输入网络，进行分类，返回四个运动类别的其中一个
    //我期望网络能学习到这样的特征：敌方该车辆先进行了快速的顺时针小陀螺，但一段时间后又不进行旋转了，由于返回的运动状态要用于对未来的预测，所以应返回的状态应该是else...
    MoveStatusSuspect TrackingZone::getMoveStatusFromSlidingWindow(std::vector<std::shared_ptr<TrackingGate2D>>& gates) {
        call_count++;
        std::vector<std::vector<double>> armors_vector {{0.0, 0.0, -1.0}, {0.0, 0.0, -1.0}, {0.0, 0.0, -1.0}, {0.0, 0.0, -1.0}};
        
        for (const auto& gate_ptr : gates) {
            const TrackingGate2D& gate = *gate_ptr;
            if (gate.armor_index != -1) {
                armors_vector.at(gate.armor_index).at(0) = gate.velocity[0];
                armors_vector.at(gate.armor_index).at(1) = gate.delta_area;
                int status_value;
                if(gate.gate_status == GateStatus::STABLE_GATE || gate.gate_status == GateStatus::UNSTABLE_LOSS_GATE)
                {
                    status_value = 1;
                }
                armors_vector.at(gate.armor_index).at(2) = static_cast<double>(status_value);
            }
        }

        // 这一帧的1*12的向量
        std::vector<double> vector_this_frame;
        for(auto& v : armors_vector)
        {
            vector_this_frame.push_back(v.at(0));
            vector_this_frame.push_back(v.at(1));
            vector_this_frame.push_back(v.at(2));
        }

        // 滑窗还没有填满
        if(call_count < WINDOW_LENGTH){
            index_sliding_window.push_back(vector_this_frame);
            return MoveStatusSuspect::UNKNOW;
        }
        else//滑窗被填满，更新
        {
            index_sliding_window.pop_front();
            index_sliding_window.push_back(vector_this_frame);

            // 每隔 length/2 次调用时使用网络预测状态
            if (call_count % (WINDOW_LENGTH / 2) == 0) {
                // 保存成csv文件
                if(this->record_flag)
                {
                    std::string csv_path = "/root/24/utils/csv_file/anti_clock_wise/output_"+std::to_string(save_count)+".csv";
                    save_count++;
                    COUT(".............................................SAVE COUNT: "<<save_count,MAGENTA);
                    std::ofstream csvfile; 
                    csvfile.open(csv_path,std::ios::out);
                    for(int i = 0; i < index_sliding_window.size(); i++)
                    {
                        for(int j = 0; j < 12; j++)
                        {
                            csvfile << index_sliding_window.at(i)[j] << ",";
                            if(j == 11)
                            {
                                csvfile << "\n";
                            }
                        }
                    }
                    csvfile.close();
                }
                // 使用你的深度学习模型进行预测
                MoveStatusSuspect predicted_status = MoveStatusSuspect::UNKNOW;//= your_network_predict_function(networkInput);
                this->last_suspect_move_status = predicted_status;

                // 返回预测的状态
                return predicted_status;
            }
            else {
                // 其他时间直接返回上一次的状态
                return last_suspect_move_status; 
            }
        }
    }

    void TrackingZone::setConfusingFactor(TrackingGate2D& gate)
    {

    }

    ///////////////TrackingGate2D///////////////
    TrackingGate2D::TrackingGate2D(const BBox& bbox,const long& time_stamp)
    {
        gate_kf = std::make_unique<KF_2D_Gate>();
        
        this->update_count = 0;

        this->gate_status = GateStatus::UNSTABLE_BUMP_GATE;

        //init KF
        Eigen::Vector3d measure_temp;
        measure_temp[0] = bbox.center.x;
        measure_temp[1] = bbox.center.y;
        measure_temp[2] = bbox.area;
        this->gate_kf->initKF(measure_temp);

        this->latest_bbox_inside = bbox;
        this->latest_time_stamp_to_be_updated = time_stamp;

        this->is_indexed = false;
        this->tag_id = bbox.tag_id;
        this->armor_index = -1;

        this->position << measure_temp[0], measure_temp[1];
        this->velocity = Eigen::Vector2d(0,0);

        this->gate_loss_count = 0;
        this->gate_maintain_count = 0;
    }

    // 拷贝构造函数
    TrackingGate2D::TrackingGate2D(const TrackingGate2D& other)
        : gate_kf(other.gate_kf), update_count(other.update_count), gate_status(other.gate_status),
        latest_bbox_inside(other.latest_bbox_inside), latest_time_stamp_to_be_updated(other.latest_time_stamp_to_be_updated),
        is_indexed(other.is_indexed), tag_id(other.tag_id), position(other.position), velocity(other.velocity),
        gate_loss_count(other.gate_loss_count), gate_maintain_count(other.gate_maintain_count), armor_index(other.armor_index),
        area(other.area), delta_area(other.delta_area), last_delta_t(other.last_delta_t){}

    // 赋值运算符
    TrackingGate2D& TrackingGate2D::operator=(const TrackingGate2D& other) {
        if (this != &other) {
            gate_kf = other.gate_kf;
            update_count = other.update_count;
            gate_status = other.gate_status;
            latest_bbox_inside = other.latest_bbox_inside;
            latest_time_stamp_to_be_updated = other.latest_time_stamp_to_be_updated;
            is_indexed = other.is_indexed;
            tag_id = other.tag_id;
            position = other.position;
            velocity = other.velocity;
            area = other.area;
            delta_area = other.delta_area;
            armor_index = other.armor_index;
            gate_loss_count = other.gate_loss_count;
            gate_maintain_count = other.gate_maintain_count;
            last_delta_t = other.last_delta_t;
        }
        return *this;
    }


    Eigen::Vector2d TrackingGate2D::predictThen_getPredictedPosition(const int& call_times)
    {
        if(call_times == 0)
        {
            this->gate_kf->predict();
        }
        return this->gate_kf->getPredictedPosition();
    }

    bool TrackingGate2D::judgeNewBBox(const BBox& bbox)
    {
        // //判断输入的bbox是否在当前这个trackingGate内
        // //利用gate_kf的卡方值判断
        // Eigen::Vector2d measure_position;
        // measure_position<< bbox.center.x ,bbox.center.y;
        // double chi_squard_value = this->gate_kf->getChiSquardValueUsingFakeUpdate(measure_position);

        // if(chi_squard_value<VERIFY_THRESH)
        // {
        //     COUT("CHI SQUARD VALUE ======================="<<chi_squard_value,GREEN);
        // }
        // else
        // {
        //     COUT("CHI SQUARD VALUE ======================="<<chi_squard_value,RED);
        // }
        // return chi_squard_value<VERIFY_THRESH;

        //卡方值太容易崩，使用矩形跟踪门？
        //三个维度：面积、x、y

        //三个当前变量
        double x_now = bbox.center.x;
        double y_now = bbox.center.y;
        double area_now = bbox.area;

        //从滤波中获取跟踪门位置、大小
        double filter_area = this->gate_kf->getArea();
        Eigen::Vector2d filter_position = this->gate_kf->getPredictedPosition();

        bool area_ok, x_ok, y_ok;
        area_ok = x_ok = y_ok = false;

        if(area_now < 4.0 * filter_area && area_now > 0.25 * filter_area)
        {
            // area in gate
            area_ok = true;
        }

        if(x_now < filter_position[0] + 3 * std::sqrt(filter_area) && x_now > filter_position[0] - 3 * std::sqrt(filter_area))
        {
            // x in gate
            x_ok = true;
        }

        if(y_now < filter_position[1] + 3 * std::sqrt(filter_area) && y_now > filter_position[1] - 3 * std::sqrt(filter_area))
        {
            // y in gate
            y_ok = true;
        }

        if(area_ok && x_ok && y_ok)
        {
            COUT("IN GATE", GREEN);
            return true;
        }
        else
        {
            std::string out_gate_message;
            if(!area_ok)
            {
                out_gate_message += "AREA, ";
            }
            if(!x_ok)
            {
                out_gate_message += "X, ";
            }
            if(!y_ok)
            {
                out_gate_message += "Y";
            }
            COUT(out_gate_message + " OUT GATE", RED);
            return false;
        }

    }

    void TrackingGate2D::inputNewBBox_thenUpdate(const BBox& bbox,long& time_stamp)
    {
        //COUT("UPDATING.........................................................",GREEN);
        this->update_count++;
        this->calculateDeltaT(time_stamp);

        this->latest_time_stamp_to_be_updated = time_stamp;
        this->latest_bbox_inside = bbox;

        Eigen::Vector3d measure_temp;
        measure_temp[0] = bbox.center.x;
        measure_temp[1] = bbox.center.y;
        measure_temp[2] = bbox.area;
        this->gate_kf->update(measure_temp);

        this->position = this->gate_kf->getPredictedPosition();
        this->velocity = this->gate_kf->getVelocity();
        this->area = this->gate_kf->getArea();
        this->delta_area = this->gate_kf->getDeltaArea();
    }

    void TrackingGate2D::calculateDeltaT(const long& time_stamp)
    {
        if(this->update_count == 0){
            last_delta_t = (double)DEFAULT_DELTA_TIME/(double)1000;
        }else{
            // last_delta_t = double(update_count-1)/(double)update_count*last_delta_t + 1/(double)update_count*double(time_stamp - this->latest_time_stamp_to_be_updated)/double(1000);
            last_delta_t = double(time_stamp - this->latest_time_stamp_to_be_updated)/double(1000);
        }
    }
}