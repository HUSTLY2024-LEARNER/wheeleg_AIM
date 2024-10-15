#pragma once

#include <vector>
#include <deque>
#include <Eigen/Dense>
#include "BBoxes.h"
#include <iostream>

using namespace DETECTOR;

#define PROCESS_NOICE_X 100000000000
#define PROCESS_NOICE_Y 100000000000
#define PROCESS_NOICE_AREA 100000

namespace TRACKER
{
    //跟踪门的2D KF
    class KF_2D_Gate
    {
    public:
        KF_2D_Gate(){
            H_position << 1, 0, 0, 0, 0, 0, 
                          0, 0, 0, 1, 0, 0;

            H_area << 1, 0;

            P_position = Eigen::Matrix<double,6,6>::Identity();

            P_area = Eigen::Matrix2d::Identity();
 
            R_position << 10, 0,
                          0, 10;

            R_area = 1; 

            x_position = Eigen::Vector<double,6>::Zero();
            A_position = Eigen::Matrix<double,6,6>::Zero();
            Q_position = Eigen::Matrix<double,6,6>::Zero();
            residual_position = Eigen::Vector2d::Zero();

            x_area = Eigen::Vector<double,2>::Zero();
            A_area = Eigen::Matrix<double,2,2>::Zero();
            Q_area = Eigen::Matrix<double,2,2>::Zero();
            residual_area = 0;
        }

        void initKF(const Eigen::Vector3d& measurement)
        {
            this->x_position[0] = measurement[0];
            this->x_position[1] = 0;
            this->x_position[2] = 0;
            this->x_position[3] = measurement[1];
            this->x_position[4] = 0;
            this->x_position[5] = 0;

            this->x_area[0] = measurement[2];
            this->x_area[1] = 0;
        }

        void predict() {
            x_position = A_position * x_position;
            P_position = A_position * P_position * A_position.transpose() + Q_position;

            x_area = A_area * x_area;
            P_area = A_area * P_area * A_area.transpose() + Q_area;           
        }

        void update(const Eigen::Vector3d& measurement) {
            Eigen::Vector2d measurement_position;
            measurement_position << measurement[0] , measurement[1];

            double measurement_area = measurement[2];

            // 计算卡尔曼增益
            auto K_position = P_position * H_position.transpose() * (H_position * P_position * H_position.transpose() + R_position).inverse();

            // 更新状态
            residual_position = measurement_position - H_position * x_position;

            // std::cout<<"residual_position: "<<residual_position<<std::endl;

            x_position = x_position + K_position * residual_position;

            P_position = (Eigen::MatrixXd::Identity(6, 6) - K_position * H_position) * P_position;

            // 计算卡尔曼增益
            auto K_area = P_area * H_area.transpose() / (H_area * P_area * H_area.transpose() + R_area);
            
            // 更新状态
            residual_area = measurement_area - H_area * x_area;
            x_area = x_area + K_area * residual_area;
            P_area = (Eigen::MatrixXd::Identity(2, 2) - K_area * H_area) * P_area;
        }

        Eigen::Vector2d getPredictedPosition()  
        {
            return {this->x_position[0] ,this->x_position[3]};
        }
        
        Eigen::Vector2d getVelocity()  
        {
            Eigen::Vector2d velocity_temp;
            velocity_temp <<x_position[1] ,x_position[4];
            return velocity_temp;
        }

        double getArea()
        {
            return this->x_area[0];
        }

        double getDeltaArea()
        {
            return this->x_area[1];
        }

        double getChiSquardValueUsingFakeUpdate(Eigen::Vector2d& measurement_position)
        {
            auto fake_K = P_position * H_position.transpose() * (H_position * P_position * H_position.transpose() + R_position).inverse();
            Eigen::Vector2d fake_residual = measurement_position - H_position * x_position;
            Eigen::Vector<double,6> fake_x = x_position + fake_K * fake_residual;
            auto fake_P = (Eigen::MatrixXd::Identity(6, 6) - fake_K * H_position) * P_position;
            return fake_residual.transpose() * (H_position * fake_P * H_position.transpose() + R_position).inverse() * fake_residual;
        }

        void updateDeltaT(const double& delta_time)
        {
            this->delta_t = delta_time;
            this->setProcessNoise();
            this->setTransitionMatrix();
        }

    private:
        Eigen::Matrix<double,6,6> A_position;  // 状态转移矩阵
        Eigen::Matrix<double,2,6> H_position;  // 观测矩阵

        Eigen::Matrix<double,6,6> Q_position;  // 过程噪声协方差
        Eigen::Matrix2d R_position;  // 观测噪声协方差
        Eigen::Matrix<double,6,6> P_position;  // 估计误差协方差

        Eigen::Vector<double,6> x_position;  // 状态向量
        Eigen::Vector2d residual_position;  //残差向量

        Eigen::Matrix<double,2,2> A_area;  // 状态转移矩阵
        Eigen::Matrix<double,1,2> H_area;  // 观测矩阵

        Eigen::Matrix<double,2,2> Q_area;  // 过程噪声协方差
        double R_area;  // 观测噪声协方差
        Eigen::Matrix<double,2,2> P_area;  // 估计误差协方差
        

        Eigen::Vector<double,2> x_area;  // 状态向量
        double residual_area;  //残差向量

        double delta_t; //更新间隔

        void setTransitionMatrix()
        {
            Eigen::Matrix2d transition_1;
            transition_1 << 1, this->delta_t, 0, 1;
            A_area = transition_1;

            Eigen::Matrix3d transition_2;
            transition_2 << 1, this->delta_t, 0.5 * this->delta_t * this->delta_t, 
                            0, 1, this->delta_t,
                            0, 0, 1;
            for(int i = 0; i < 2; i++)
            {
                A_position.block<3, 3>(i * 3, i * 3) = transition_2;
            }
        }

        void setProcessNoise()
        {
            Eigen::Vector2d process_noise_vec_CV;
            process_noise_vec_CV << 0.5 * delta_t * delta_t, delta_t;
            Eigen::Matrix<double,2,1> process_noise_matrix_area;
            process_noise_matrix_area = process_noise_vec_CV;

            Eigen::Matrix3d process_noise_matrix_CA;
            process_noise_matrix_CA << std::pow(delta_t, 5) / 20.0, std::pow(delta_t, 4) / 8.0, std::pow(delta_t, 3) / 6.0,
                                       std::pow(delta_t, 4) / 8.0, std::pow(delta_t, 3) / 3.0, std::pow(delta_t, 2) /2.0,
                                       std::pow(delta_t, 3) / 6.0, std::pow(delta_t, 2) /2.0, delta_t;

            this->Q_position.block<3, 3>(0, 0) = PROCESS_NOICE_X * process_noise_matrix_CA;
            this->Q_position.block<3, 3>(3, 3) = PROCESS_NOICE_Y * process_noise_matrix_CA;

            double process_noice_area = PROCESS_NOICE_AREA;
            this->Q_area = process_noise_matrix_area * process_noice_area *process_noise_matrix_area.transpose();
        }
    };

    //STABLE , UNSTABLE_BUMP , UNSTABLE_LOSS , DIED
    enum class GateStatus
    {
        STABLE_GATE,
        UNSTABLE_BUMP_GATE,
        UNSTABLE_LOSS_GATE,
        DIED_GATE
    };

    //跟踪门
    class TrackingGate2D
    {
    public:
        TrackingGate2D(const BBox& bbox,const long& time_stamp);
        TrackingGate2D(const TrackingGate2D& other);
        TrackingGate2D& operator=(const TrackingGate2D& other);

        Eigen::Vector2d predictThen_getPredictedPosition(const int& call_times);
        bool judgeNewBBox(const BBox& bbox);
        void inputNewBBox_thenUpdate(const BBox& bbox,long& time_stamp);
        void calculateDeltaT(const long& time_stamp);



        std::shared_ptr<KF_2D_Gate> gate_kf;
        GateStatus gate_status;
        Eigen::Vector2d position;
        Eigen::Vector2d velocity;
        double area;
        double delta_area;
        BBox latest_bbox_inside;
        bool is_indexed;
        int armor_index;
        int tag_id;
        long latest_time_stamp_to_be_updated;
        int gate_loss_count,gate_maintain_count;
        int update_count;
        double last_delta_t;  

    };

    typedef std::vector<TrackingGate2D> TrackingGates;

    //STABLE , UNSTABLE , DIED
    enum class ZoneStatus
    {
        STABLE_ZONE,
        UNSTABLE_ZONE,
        DIED_ZONE
    };

    enum class MoveStatusSuspect
    {
        UNKNOW,
        RELATIVE_STAND,
        RELATIVE_TRANSLATE,
        CLOCKWISE_SPIN,
        COUNTER_CLOCKWISE_SPIN,
        TWISTING
    };

    class TrackingZone
    {
    public:
        TrackingZone() : initialized(false), call_count(0), save_count(0), record_flag(false), gates_in_zone({}), zone_center(Eigen::Vector2d::Zero()), zone_status(ZoneStatus::DIED_ZONE) {}
        TrackingZone(std::vector<std::shared_ptr<TrackingGate2D>>& gates);

        operator bool() const {return this->initialized;}
        bool initializeState() const{return this->initialized;};
        void destroyItself() {this->initialized = false; this->zone_status = ZoneStatus::DIED_ZONE; this->call_count = 0; this->is_confusing = false; this->gates_in_zone.clear();}
        MoveStatusSuspect getMoveStatusFromSlidingWindow(std::vector<std::shared_ptr<TrackingGate2D>>& gates);
        void setConfusingFactor(TrackingGate2D& gate);
        void setRecordFlag(const bool& flag) {this->record_flag = flag;};
        int zone_tag_id; //zone对应的装甲板数字id

        //STABLE , UNSTABLE , DIE
        ZoneStatus zone_status;

        //UNKNOW , RELATIVE_STAND , RELATIVE_TRANSLATE , CLOCKWISE_SPIN , COUNTER_CLOCKWISE_SPIN , TWISTING 
        MoveStatusSuspect suspect_move_status;
        Eigen::Vector2d zone_center;

        std::vector<std::shared_ptr<TrackingGate2D>> gates_in_zone;
    private:
        bool is_confusing;
        bool initialized;
        bool record_flag;
        MoveStatusSuspect last_suspect_move_status;
        std::deque<std::vector<double>> index_sliding_window;
        unsigned long call_count; //不会溢出，放心
        unsigned long save_count;
    };

    //每个数字类别单独一个tracker，内部存放该类别的跟踪门
    class SingleTypeTracker {
    public:
        SingleTypeTracker(const int& id) : single_tag_id(id), gate_base_exist(false), single_type_gates(std::vector<std::shared_ptr<TRACKER::TrackingGate2D>>()), single_type_zone(std::make_unique<TrackingZone>())
        , single_type_gates_copy(std::vector<std::shared_ptr<TRACKER::TrackingGate2D>>()), died_numbered_gates(std::deque<std::shared_ptr<TRACKER::TrackingGate2D>>()) {}

        // 返回该SingleTypeTracker的跟踪域
        const std::shared_ptr<TrackingZone> getTrackingZone() const { return this->single_type_zone; };

        void inputSingleTypeMeasurements(const BBoxes& bboxes, long& time_stamp);
        void updateTimeStamp(const long& time_stamp);
        void mainTainTrackingGates(const bool& detection_loss);
        void setRecordFlag(const bool& flag) { this->single_type_zone->setRecordFlag(flag); };

    private:
        bool numberingAllGates();
        void maintainTrackingZone(const bool& empty_flag);
        int findClosestGateID(const BBox& bbox, const int& call_times);
        void createNewTrackingGate(const BBox& bbox);
        bool ifGateInZone(const TrackingGate2D& gate);
        void updateLossGatesPosition();

        std::vector<std::shared_ptr<TrackingGate2D>> single_type_gates;
        std::vector<std::shared_ptr<TrackingGate2D>> single_type_gates_copy;
        std::deque<std::shared_ptr<TrackingGate2D>> died_numbered_gates;
        std::shared_ptr<TrackingZone> single_type_zone;
        bool gate_base_exist = false;
        long time_stamp;
        int single_tag_id;
    };

    class TargetsTracker
    {
    public:
        TargetsTracker();
        const std::vector<std::shared_ptr<TrackingZone>> getTrackingZones() const;
        const std::vector<std::shared_ptr<TrackingZone>> getStableTrackingZones() const;
        void inputNewMeasurements(const BBoxes& bboxes,long &time_stamp);
        void remindAllTrackersTimeOut();
        void setRecordFlag(const bool& flag);
    private:
        //0:base 1:hero 2:engineer 3:infantry3 4:infantry4 5:infantry5 6:sentry 7:outpost
        std::vector<SingleTypeTracker> trackers;

        double detectionGateSize;
    };

}