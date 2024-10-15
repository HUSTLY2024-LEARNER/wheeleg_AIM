#include "MultiPredictor.hpp"

#include "umt.hpp"

#define FOLLOW_THRESH_1 6.0
#define FOLLOW_THRESH_2 10.0

#define CHOOSE_0_2 true

namespace PREDICTOR
{
    VehiclePredictor::VehiclePredictor()
    {
        this->rpm_estimator = std::make_unique<RpmEstimator>();
        this->center_state_estimator = std::make_unique<CenterStateEstimator>();
        this->linear_estimator = std::make_unique<LinearEstimator>();
    }

    void VehiclePredictor::resetPredictor()
    {
        this->is_kalman_init = false;
    }

    void VehiclePredictor::rebootPredictor()
    {
        this->resetPredictor();
    }
    
    void VehiclePredictor::runPredictor(SOLVER::IndexedArmorPoses& indexed_armor_poses,TRACKER::MoveStatusSuspect& move_status,double& update_time)
    {
        if(!is_kalman_init)
        {
            is_kalman_init = true;
            rebootAllEstimator(indexed_armor_poses);
        }
        else
        {
            // LINEAR
            this->linear_estimator->setT(update_time);
            this->linear_estimator->inputMeasurement(indexed_armor_poses);
            // Eigen::Vector3d ca_predicted_position, cv_predicted_position, static_predicted_position;
            // ca_predicted_position = this->linear_estimator->getMessageCA().at(0) + this->linear_estimator->getMessageCA().at(1) * update_time + 0.5 * this->linear_estimator->getMessageCA().at(2) * update_time * update_time;
            // cv_predicted_position = this->linear_estimator->getMessageCV().at(0) + this->linear_estimator->getMessageCV().at(1) * update_time;
            // static_predicted_position = this->linear_estimator->getMessageStatic();

            this->linear_predicted_position = this->linear_estimator->getMessageSphereCV().at(0);
            this->linear_predicted_speed = this->linear_estimator->getMessageSphereCV().at(1);
            this->linear_predicted_acc = {0, 0, 0};

            COUT("LINEAR OK:"<<linear_predicted_position, GREEN);

            // RPM
            this->rpm_estimator->setT(update_time);
            this->rpm_estimator->inputMeasurement(indexed_armor_poses);
            this->angular_speed = this->rpm_estimator->getAngularSpeed();
            this->statistical_angular_state = this->rpm_estimator->getStatisticalAngularState();
            this->two_angle = this->rpm_estimator->getDoubleAngle();
            COUT("RPM OK :"<<two_angle[0]<<"  "<<two_angle[1], GREEN);

            // CENTER STATE
            this->center_state_estimator->setT(update_time);
            this->center_state_estimator->inputMeasurement(indexed_armor_poses, two_angle);
            this->car_center_position = this->center_state_estimator->getCarCenterPosition();
            this->car_center_speed = this->center_state_estimator->getCarCenterSpeed();
            this->car_radius_proportion = this->center_state_estimator->getRadiusProportion();
            this->armor_clearance = this->center_state_estimator->getArmorClearance();
            COUT("CENTER STATE OK :", GREEN);
        }
    }

    void VehiclePredictor::rebootAllEstimator(SOLVER::IndexedArmorPoses indexed_poses)
    {
        this->rpm_estimator->rebootEstimator(indexed_poses);
        this->center_state_estimator->rebootEstimator(indexed_poses);
        this->linear_estimator->rebootEstimator(indexed_poses);
    }

    FollowMode VehiclePredictor::getFollowMode()
    {
        std::cout<<"speed:"<<statistical_angular_state.second<<std::endl;
        // LINEAR
        if(this->statistical_angular_state.first == UNKNOW)
        {
            COUT("FOLLOW MODE: LINEAR", YELLOW);
            return FollowMode::LINEAR_TRACKING;
        }
        else
        {
            // 从装甲板切换频率来确定进入DENSE还是SPARSE或STAND
            if(std::abs(this->statistical_angular_state.second) < FOLLOW_THRESH_1)
            {
                COUT("FOLLOW MODE: DENSE", YELLOW); 
                return FollowMode::DENSE_TRACKING;
            }
            else if(std::abs(this->statistical_angular_state.second) >= FOLLOW_THRESH_1 && std::abs(this->statistical_angular_state.second) <= FOLLOW_THRESH_2)
            {
                COUT("FOLLOW MODE: SPARSE", YELLOW);
                return FollowMode::DENSE_TRACKING;
            }
            else if(std::abs(this->statistical_angular_state.second) > FOLLOW_THRESH_2)
            {
                COUT("FOLLOW MODE: STAND", YELLOW);
                return FollowMode::DENSE_TRACKING;
            }
        }
    }

    SpinState VehiclePredictor::getSpinState()
    {
        switch (this->rpm_estimator->getSpinState())
        {
        case 0:
            return SpinState::UNKNOW_STATE;
            break;
        case 1:
            return SpinState::CLOCK_WISE_STATE;
            break;
        case -1:
            return SpinState::COUNTER_CLOCK_WISE_STATE;
            break;
        default:
            return SpinState::UNKNOW_STATE;
            break;
        }
    }

    std::vector<Eigen::Vector3d> VehiclePredictor::getAimPointsAfterBulletFly(const float& delay_t, FollowMode follow_mode)
    {
        Eigen::Vector4d car_center_speed_expend = {this->car_center_speed[0], this->car_center_speed[1], this->car_center_speed[2], this->car_center_speed[2]};
        this->predicted_car_center = this->car_center_position + car_center_speed_expend * delay_t;

        double predicted_angle_0 = this->two_angle[0] + this->angular_speed * delay_t;
        double predicted_angle_1 = this->two_angle[1] + this->angular_speed * delay_t;
        double radius_0 = this->armor_clearance * this->car_radius_proportion;
        double radius_1 = this->armor_clearance * std::sqrt(1 - this->car_radius_proportion * this->car_radius_proportion);
        
        umt::Publisher<std::vector<Eigen::Vector3d>> points_pub("points_pre");
        std::vector<Eigen::Vector3d> send_points_temp;
        send_points_temp.push_back({predicted_car_center[0] - radius_0 * sin(predicted_angle_0), predicted_car_center[1] - radius_0 * cos(predicted_angle_0), predicted_car_center[2]});
        send_points_temp.push_back({predicted_car_center[0] - radius_1 * sin(predicted_angle_0 + M_PI_2), predicted_car_center[1] - radius_1 * cos(predicted_angle_0 + M_PI_2), predicted_car_center[3]});
        send_points_temp.push_back({predicted_car_center[0] - radius_0 * sin(predicted_angle_0 + M_PI), predicted_car_center[1] - radius_0 * cos(predicted_angle_0 + M_PI), predicted_car_center[2]});
        send_points_temp.push_back({predicted_car_center[0] - radius_1 * sin(predicted_angle_0 + 3 * M_PI_2), predicted_car_center[1] - radius_1 * cos(predicted_angle_0 + 3 * M_PI_2), predicted_car_center[3]});
        send_points_temp.push_back({predicted_car_center[0], predicted_car_center[1], predicted_car_center[2]});

        // std::cout<<"center:"<<predicted_car_center<<std::endl;
        // send_points_temp.push_back({predicted_car_center[0], predicted_car_center[1], predicted_car_center[2]});
        points_pub.push(send_points_temp);

        std::vector<Eigen::Vector3d> aim_points_temp;

        if(follow_mode == FollowMode::LINEAR_TRACKING)
        {
           aim_points_temp.push_back(linear_predicted_position + linear_predicted_speed * delay_t + linear_predicted_acc * delay_t * delay_t * 0.5);
        }
        else if(follow_mode == FollowMode::DENSE_TRACKING)
        {
            // 此时角速度较小，返回所有四块装甲板
            aim_points_temp.push_back({predicted_car_center[0] - radius_0 * sin(predicted_angle_0), predicted_car_center[1] - radius_0 * cos(predicted_angle_0), predicted_car_center[2]});
            aim_points_temp.push_back({predicted_car_center[0] - radius_1 * sin(predicted_angle_0 + M_PI_2), predicted_car_center[1] - radius_1 * cos(predicted_angle_0 + M_PI_2), predicted_car_center[3]});
            aim_points_temp.push_back({predicted_car_center[0] - radius_0 * sin(predicted_angle_0 + M_PI), predicted_car_center[1] - radius_0 * cos(predicted_angle_0 + M_PI), predicted_car_center[2]});
            aim_points_temp.push_back({predicted_car_center[0] - radius_1 * sin(predicted_angle_0 + 3 * M_PI_2), predicted_car_center[1] - radius_1 * cos(predicted_angle_0 + 3 * M_PI_2), predicted_car_center[3]});
        }
        else if(follow_mode == FollowMode::SPARSE_TARACKING)
        {
            // 此时角速度较大，但还在云台相应范围内，返回某两块对侧装甲板
            if(CHOOSE_0_2)
            {
                aim_points_temp.push_back({predicted_car_center[0] - radius_0 * sin(predicted_angle_0), predicted_car_center[1] - radius_0 * cos(predicted_angle_0), predicted_car_center[2]});
                aim_points_temp.push_back({predicted_car_center[0] - radius_0 * sin(predicted_angle_0 + M_PI), predicted_car_center[1] - radius_0 * cos(predicted_angle_0 + M_PI), predicted_car_center[2]});

            }
            else
            {
                aim_points_temp.push_back({predicted_car_center[0] - radius_1 * sin(predicted_angle_0 + M_PI_2), predicted_car_center[1] - radius_1 * cos(predicted_angle_0 + M_PI_2), predicted_car_center[3]});
                aim_points_temp.push_back({predicted_car_center[0] - radius_1 * sin(predicted_angle_0 + 3 * M_PI_2), predicted_car_center[1] - radius_1 * cos(predicted_angle_0 + 3 * M_PI_2), predicted_car_center[3]});
            }
        }
        else if(follow_mode == FollowMode::STAND_WAITING)
        {
            // 此时角速度太大，定点等待装甲板到来，返回所有四块装甲板
            aim_points_temp.push_back({predicted_car_center[0] - radius_0 * sin(predicted_angle_0), predicted_car_center[1] - radius_0 * cos(predicted_angle_0), predicted_car_center[2]});
            aim_points_temp.push_back({predicted_car_center[0] - radius_1 * sin(predicted_angle_0 + M_PI_2), predicted_car_center[1] - radius_1 * cos(predicted_angle_0 + M_PI_2), predicted_car_center[3]});
            aim_points_temp.push_back({predicted_car_center[0] - radius_0 * sin(predicted_angle_0 + M_PI), predicted_car_center[1] - radius_0 * cos(predicted_angle_0 + M_PI), predicted_car_center[2]});
            aim_points_temp.push_back({predicted_car_center[0] - radius_1 * sin(predicted_angle_0 + 3 * M_PI_2), predicted_car_center[1] - radius_1 * cos(predicted_angle_0 + 3 * M_PI_2), predicted_car_center[3]});
        }
        
        std::cout<<"aim size"<<aim_points_temp.size()<<std::endl;
        return aim_points_temp;
    }

    CenterAndRange VehiclePredictor::getAimBounds()
    {
        double radius_0 = this->armor_clearance * this->car_radius_proportion;
        double radius_1 = this->armor_clearance * std::sqrt(1 - this->car_radius_proportion * this->car_radius_proportion);

        Eigen::Vector3d aim_center = {this->predicted_car_center[0], this->predicted_car_center[1], this->predicted_car_center[2]};

        return {aim_center, 1.2 * std::max(radius_0, radius_1)};
    }
}
