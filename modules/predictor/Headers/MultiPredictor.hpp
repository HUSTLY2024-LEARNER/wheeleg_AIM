#pragma once

#include <vector>
#include <memory>
#include "solver.hpp"
#include "tracker.hpp"
#include "utils.h"
#include "RpmEstimator.hpp"
#include "CenterStateEstimator.hpp"
#include "LinearEstimator.hpp"
#include "AngleTimeSolver.hpp"

namespace PREDICTOR
{
#define DEFAULT_DELTA_TIME 10

    class MultiArmorPredictor
    {
    public:
        MultiArmorPredictor() {}
        virtual ~MultiArmorPredictor() {}

        //delta_time所有predictor类都公用,这样保证某些类掉帧的时候仍能按照正确的delta_time做不量测更新
        static double updateDeltaTime(const long& time_stamp)
        {
            static int update_count = 0;
            static long last_time_stamp;
            double average_delta_time;
            if(update_count == 0){
                average_delta_time = (double)DEFAULT_DELTA_TIME/(double)1000;
                last_time_stamp = time_stamp;
            }else{
                // average_delta_time = ((double(update_count-1)/(double)update_count)*average_delta_time + (1/(double)update_count)*(double(time_stamp - last_time_stamp)))/double(1000);
                average_delta_time = (double(time_stamp - last_time_stamp))/double(1000);
                std::cout<<"delta time:"<<average_delta_time<<std::endl;
                last_time_stamp = time_stamp;
            }
            update_count++;
            return average_delta_time;
        }
        
        virtual void rebootPredictor()=0;
        virtual void runPredictor(SOLVER::IndexedArmorPoses& armor_poses,TRACKER::MoveStatusSuspect& move_status,double& update_time)=0;
        virtual std::vector<Eigen::Vector3d> getAimPointsAfterBulletFly(const float& delay_t, FollowMode follow_mode)=0;
        virtual FollowMode getFollowMode()=0;
        virtual SpinState getSpinState()=0;
        virtual CenterAndRange getAimBounds()=0;
    protected:
        

    };

    class OutpostPredictor : public MultiArmorPredictor
    {
        void rebootPredictor() override;
        void runPredictor(SOLVER::IndexedArmorPoses& armor_poses,TRACKER::MoveStatusSuspect& move_status,double& update_time) override;    
        std::vector<Eigen::Vector3d> getAimPointsAfterBulletFly(const float& delay_t, FollowMode follow_mode) override;
        FollowMode getFollowMode() override;
        SpinState getSpinState() override;
        CenterAndRange getAimBounds() override;
    };

    class VehiclePredictor : public MultiArmorPredictor
    {
    public:
        VehiclePredictor();
        void rebootPredictor() override;
        void runPredictor(SOLVER::IndexedArmorPoses& armor_poses,TRACKER::MoveStatusSuspect& move_status,double& update_time) override;    
        std::vector<Eigen::Vector3d> getAimPointsAfterBulletFly(const float& delay_t, FollowMode follow_mode) override;
        FollowMode getFollowMode() override;
        SpinState getSpinState() override;
        CenterAndRange getAimBounds() override;
    private:
        std::unique_ptr<RpmEstimator> rpm_estimator;
        std::unique_ptr<CenterStateEstimator> center_state_estimator;
        std::unique_ptr<LinearEstimator> linear_estimator;

        bool is_kalman_init;
        double angular_speed;
        std::pair<int,double> statistical_angular_state;
        double car_radius_proportion;
        double armor_clearance;
        Eigen::Vector2d two_angle;
        Eigen::Vector4d car_center_position;
        Eigen::Vector3d car_center_speed;
        Eigen::Vector4d predicted_car_center;

        Eigen::Vector3d linear_predicted_position;
        Eigen::Vector3d linear_predicted_speed;
        Eigen::Vector3d linear_predicted_acc;
        
        void rebootAllEstimator(SOLVER::IndexedArmorPoses);
    public:
        void resetPredictor();
        void setUpdateTime(double update_time);
    };

    class NotPredictor : public MultiArmorPredictor
    {
        void rebootPredictor() override;
        void runPredictor(SOLVER::IndexedArmorPoses& armor_poses,TRACKER::MoveStatusSuspect& move_status,double& update_time) override;
        std::vector<Eigen::Vector3d> getAimPointsAfterBulletFly(const float& delay_t, FollowMode follow_mode) override;
        FollowMode getFollowMode() override;
        SpinState getSpinState() override;
        CenterAndRange getAimBounds() override;
    private:
        std::vector<Eigen::Vector3d> my_points;
    };

    class PredictorFactory
    {
    public:
        static std::unique_ptr<MultiArmorPredictor> createPredictor(const std::string &type)
        {
            if(type == "Outpost"){
                return std::make_unique<OutpostPredictor>();
            } else if(type == "Vehicle"){
                return std::make_unique<VehiclePredictor>();
            } else if(type == "Not"){
                return std::make_unique<NotPredictor>();
            } else{
                return std::make_unique<NotPredictor>();
            }
        }

        static std::unique_ptr<MultiArmorPredictor> createPredictor(const ENEMY_TYPE &type, const ARMOR_SIZE &size)
        {
            if(type == ENEMY_TYPE::Hero || (((type == ENEMY_TYPE::Infantry3)||(type == ENEMY_TYPE::Infantry4)||(type == ENEMY_TYPE::Infantry5))&&(size == ARMOR_SIZE::SMALL_ARMOR)) || type == ENEMY_TYPE::Engineer || type == ENEMY_TYPE::Sentry){
                //return std::make_unique<QuadrupleArmorPredictor>();
                return std::make_unique<NotPredictor>();
            } else if(((type == ENEMY_TYPE::Infantry3)||(type == ENEMY_TYPE::Infantry4)||(type == ENEMY_TYPE::Infantry5))&&(size == ARMOR_SIZE::BIG_ARMOR)){
                //return std::make_unique<DoubleArmorPredictor>();
                return std::make_unique<NotPredictor>();
            } else if(type == ENEMY_TYPE::Outpost){
                //return std::make_unique<TripleArmorPredictor>();
                return std::make_unique<NotPredictor>();
            } else if(type == ENEMY_TYPE::Base){
                return std::make_unique<NotPredictor>();
            } else{
                return std::make_unique<NotPredictor>();
            }
        }
    };
    
} //namespace PREDICTOR