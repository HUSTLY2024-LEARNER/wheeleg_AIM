#include <thread>
#include <umt.hpp>

#include <predictor.hpp>
#include <solver.hpp>
#include <MultiPredictor.hpp>
#include <FireController.hpp>
#include <TimedGimbalTask.hpp>
#include <AngleTimeSolver.hpp>

#include <utils.h>

using namespace LY_UTILS;

//#define IS_USE_PREDICT
#define delay_time 0.05

namespace PREDICTOR
{
    void predictor_run(const bool is_use_predict) 
    {
        umt::Subscriber<SOLVER::SolutionPackage> solution_sub("solution_pack");
        umt::Publisher<PredictionPackage> prediction_pub("prediction_pack");
        
        std::map<ENEMY_TYPE, std::unique_ptr<MultiArmorPredictor>> predictors;

        std::unique_ptr<FireController> fire_controller = std::make_unique<FireController>();
        std::unique_ptr<AngleTimeSolver> angle_time_solver = std::make_unique<AngleTimeSolver>();

        auto armor_size_map = umt::ObjManager<ARMOR_SIZE_MAP>::find_or_create("armor_size_map");
        ENEMY_TYPE last_enemy_type = ENEMY_TYPE::UNKNOW_TYPE;

        auto imu_flag_data = umt::ObjManager<DRIVER::SerialReadData::IMU_Flag>::find_or_create("imu_flag_data");

        int predictor_loss_count = 0;

        while(true)
        {
            try{
                auto solution_pack = solution_sub.pop_for(1000); 
                auto *this_frame_armor_size_map = armor_size_map.get();
                auto *this_frame_imu_flag_data = imu_flag_data.get();

                // 打印解算的yaw角
                //COUT("YAW_WORLD "<<solution_pack.armor_poses.at(0).second.yaw_world,RED);

                predictor_loss_count = 0;
                // angle_time_solver->calculateAverageBulletSpeed(solution_pack.imu_flag.actual_bullet_speed);

                auto it = predictors.find(solution_pack.enemy_type);

                if(solution_pack.enemy_type != last_enemy_type)
                {
                    auto it_other = predictors.find(last_enemy_type);
                    if(it != predictors.end())
                    {
                        it->second->rebootPredictor();
                    }
                }

                last_enemy_type = solution_pack.enemy_type;

                if (it != predictors.end()) {
                    // predictor已存在，可以使用
                    double update_time = it->second->updateDeltaTime(solution_pack.time_stamp);
                    it->second->runPredictor(solution_pack.armor_poses,solution_pack.move_status,update_time);

                    // 获取跟随模式：线性跟随、紧密陀螺跟随、稀疏陀螺跟随，定点等待
                    FollowMode follow_mode = it->second->getFollowMode();
                    SpinState spin_state = it->second->getSpinState();

                    float fly_time = angle_time_solver->getBulletFlyTime(solution_pack.armor_poses.at(0).second.translate);

                    // // TODO:单点和连发的delay_time不一样
                    std::vector<Eigen::Vector3d> after_bullet_fly_points = it->second->getAimPointsAfterBulletFly(fly_time + delay_time, follow_mode);
                    CenterAndRange aim_bounds = it->second->getAimBounds();
                    auto yaw_and_distance_bounds = angle_time_solver->getGoalArea(aim_bounds, spin_state);
                    
                    std::pair<YawAndPitchRegion,Distance> yaw_and_pitch_region_with_distance = angle_time_solver->getYawPitchRegion(after_bullet_fly_points, this_frame_armor_size_map->at(solution_pack.enemy_type), yaw_and_distance_bounds, follow_mode, spin_state);

                    YawAndPitch aim_yaw_pitch = fire_controller->getAimYawPitch(this_frame_imu_flag_data->yaw_now, this_frame_imu_flag_data->pitch_now, yaw_and_pitch_region_with_distance, yaw_and_distance_bounds);

                    // 打印云台的yaw
                    COUT("IMU_YAW "<<this_frame_imu_flag_data->yaw_now,RED);

                    uint8_t shoot_flag = fire_controller->getShootFlag(aim_yaw_pitch, std::make_pair((float)this_frame_imu_flag_data->yaw_now, (float)this_frame_imu_flag_data->pitch_now), follow_mode); 

                    aim_yaw_pitch.first = (float)(aim_yaw_pitch.first + std::round((solution_pack.imu_flag.yaw_now - aim_yaw_pitch.first) / 360.0) * 360.0);
                    prediction_pub.push({static_cast<uint8_t>(solution_pack.enemy_type), shoot_flag, aim_yaw_pitch.second, aim_yaw_pitch.first});
                } else {
                    // predictor 不存在，创建一个
                    std::unique_ptr<MultiArmorPredictor> new_predictor = nullptr;
                    if(is_use_predict){
                        // TODO
                        new_predictor = PredictorFactory::createPredictor("Vehicle");
                    } else{
                        // 所有的enemy_type都创建NotPredictor
                        new_predictor = PredictorFactory::createPredictor("Not");
                    }
                    if (new_predictor) {
                        predictors[solution_pack.enemy_type] = std::move(new_predictor);
                        auto& predictor = predictors[solution_pack.enemy_type];
                        // 初始化新创建的predictor
                        double update_time = predictor->updateDeltaTime(solution_pack.time_stamp);
                        
                        predictor->runPredictor(solution_pack.armor_poses, solution_pack.move_status,update_time);
                    }
                }
            } catch (umt::MessageError_Timeout &e) {
                //规定时限内未获取结果
                predictor_loss_count ++;
                if(predictor_loss_count > 100)
                {
                    auto it = predictors.find(last_enemy_type);
                    if(it != predictors.end())
                    {
                        it->second->rebootPredictor();
                    }
                }
                //COUT("[WARNING] 'solution_pack' "<<e.what(),RED);
            } catch (umt::MessageError &e) {
                COUT("[WARNING] 'track_pack' "<<e.what(),RED);
            }
        }
    }
    
    bool timed_gimbal_task()
    {
        VariableLocker<uint8_t> shoot_flag_locker;

        umt::Subscriber<PredictionPackage> prediction_sub("prediction_pack",0);
        umt::Publisher<DRIVER::SerialWriteData> serial_write_data_pub("serial_write");
        std::unique_ptr<TimedGimbalTask> gimbal_task = std::make_unique<TimedGimbalTask>();

        // auto aim_offset = umt::ObjManager<std::pair<float, float>>::find_or_create("aim_offset");
        // *aim_offset.get() = std::make_pair(,);
        auto center_state = umt::ObjManager<Eigen::Matrix<double, 8, 1>>::find_or_create("center_state");
        
        uint8_t lock_test = 0;

        //UDPSender udp_debug("192.168.137.1", 3000);

        while(true){
            try {
                auto predict_pack = prediction_sub.pop_for(5);
                //gimbal_task->inputSendMessage();
                COUT("MESSAGE SEND, YAW: "<<predict_pack.yaw_setpoint<<"PITCH :"<<predict_pack.pitch_setpoint,GREEN);
                shoot_flag_locker.setValue(predict_pack.shoot_flag);
                if(shoot_flag_locker.getValue() == (uint8_t)1)
                {
                    COUT("SHOOT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", GREEN); 
                }
                else
                {
                    COUT(" NO SHOOT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", RED);
                }

                // serial_write_data_pub.push({'!', predict_pack.detect_number, shoot_flag_locker.getValue(), predict_pack.pitch_setpoint - 1.0f, predict_pack.yaw_setpoint + 0.0f});
                serial_write_data_pub.push({'!', 1, shoot_flag_locker.getValue(), predict_pack.pitch_setpoint, predict_pack.yaw_setpoint});
                auto center_state = umt::ObjManager<Eigen::Matrix<double, 8, 1>>::find_or_create("center_state");
                
                Eigen::Matrix<double, 8, 1> center_state_matrix = *center_state.get();
/*
                CenterStateFrame center_state_frame;
                center_state_frame.x_c = center_state_matrix(0, 0);
                center_state_frame.v_x = center_state_matrix(1, 0);
                center_state_frame.y_c = center_state_matrix(2, 0);
                center_state_frame.v_y = center_state_matrix(3, 0);
                center_state_frame.z_1 = center_state_matrix(4, 0);
                center_state_frame.z_2 = center_state_matrix(5, 0);
                center_state_frame.v_z = center_state_matrix(6, 0);
                center_state_frame.k = center_state_matrix(7, 0);
                udp_debug.send(center_state_frame);
                
                // SendFrame send_frame;
                // send_frame.vaild = 1;
                // send_frame.shootflag = shoot_flag_locker.getValue();
                // send_frame.pitch = predict_pack.pitch_setpoint;
                // send_frame.yaw = predict_pack.yaw_setpoint;
                // udp_debug.send(send_frame);
  */          
            } catch (umt::MessageError_Timeout &e) {
                //规定时限内未获取识别结果
                serial_write_data_pub.push({'!', (u_int8_t)0, shoot_flag_locker.getValue(), (float)999.0f, (float)999.0f});

                //serial_write_data_pub.push(gimbal_task->getPredictSendMessage());
            } catch (umt::MessageError &e) {
                COUT("MESSAGE SEND RESTART", RED);
                return false;
            }
        }
        return false;
    }

    void bkg_predictor_run(const bool& is_use_predict){
        std::thread([is_use_predict](){
            predictor_run(is_use_predict);
        }).detach();
    }

    void bkg_sender_run(){
        using namespace std::chrono_literals;
        std::thread([](){
            while(!timed_gimbal_task()){
                std::this_thread::sleep_for(500ms);
            }
        }).detach();
    }

PYBIND11_EMBEDDED_MODULE(PREDICTOR_, m) {
    namespace py = pybind11;
    m.def("bkg_predictor_run", bkg_predictor_run, py::arg("is_use_predict"));
    // m.def("bkg_sender_run", bkg_sender_run, py::arg("pitch_offset"), py::arg("yaw_offset"));
    m.def("bkg_sender_run", bkg_sender_run);
}

}
