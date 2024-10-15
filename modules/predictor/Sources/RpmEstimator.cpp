#include "RpmEstimator.hpp"

namespace PREDICTOR
{
    RpmEstimator::RpmEstimator()
    {
        this->yaw_filter = std::make_unique<ExtendedKalman<double,3,3>>();
        last_cloest_index = -1;
    };

    RpmEstimator::~RpmEstimator()
    {

    };

    void RpmEstimator::setT(double t)
    {
        this->update_time = t;
    }
    
    void RpmEstimator::inputMeasurement(SOLVER::IndexedArmorPoses indexed_poses)
    {
        // 需要从pose中得到的信息有：目标1或2个装甲板的yaw角、目标的装甲板切换频率
        Eigen::Vector3d yaw_measurement;
        if(indexed_poses.size() == 1) // 只有单装甲板观测
        {
            COUT("SINGLE ARMOR DETECT", RED);
            double single_yaw = indexed_poses.at(0).second.yaw_world;
            yaw_measurement = makeYawMeasurement(single_yaw,indexed_poses.at(0).first);
            
            //需要设置yaw_1和yaw_2的协方差，使二者相关
            setMeasurementNoise(indexed_poses.at(0).second.translate.norm(),false);
        }
        else // 有双装甲板观测
        {
            COUT("DOUBLE ARMOR DETECT", RED);

            yaw_measurement = makeYawMeasurement(indexed_poses.at(0).second.yaw_world,indexed_poses.at(0).first,indexed_poses.at(1).second.yaw_world,indexed_poses.at(1).first);

            setMeasurementNoise(indexed_poses.at(0).second.translate.norm(),true);
        }

        // make yaw measurement continuous
        yaw_measurement = makeYawContinues(yaw_measurement);

        // run filter
        setProcessNoise();
        setTransitionMatrix();
        yaw_filter->predict(yaw_state_to_yaw_measure, yaw_measurement);
        Eigen::Vector3d angles_and_speed = yaw_filter->update();
        double angular_speed_filter = angles_and_speed[2];

        COUT("ANGULAR SPEED:"<<angular_speed_filter, GREEN);
        double armor_switch_frequency = getArmorSwitchFrequency(indexed_poses.at(0).first);

        // 根据滤波出的角速度和频率统计的角速度综合得出角速度
        // 融合依据：距离or角度方差

        std::cout<<armor_switch_frequency<<std::endl;
        if(armor_switch_frequency != -999.9)
        {
            this->statistical_angular_speed = armor_switch_frequency * M_PI_2;
            std::cout<<"statistical speed:"<<this->statistical_angular_speed<<std::endl;
        }
        else
        {
            this->statistical_angular_speed = 0.;
        }

        this->angular_speed = angleFusion(angular_speed_filter, statistical_angular_speed);
        this->yaw_filter->posteriori_state_estimate[2] = this->angular_speed;
        std::cout<<"angle fusion:"<<this->angular_speed<<std::endl;
    };

    void RpmEstimator::setProcessNoise()
    {
        Eigen::Matrix<double, 2, 1> process_noice_vec;
        process_noice_vec << 0.5 * update_time * update_time, update_time;
        Eigen::Matrix<double,1,1> process_noice_matrix;
        process_noice_matrix.diagonal() << YAW_PROCESS_NOICE;
        Eigen::Matrix2d process_noice_single = process_noice_vec * process_noice_matrix * process_noice_vec.transpose();
        this->yaw_filter->process_noise_cov << process_noice_single(0,0), 0, process_noice_single(0,1),
                                               0, process_noice_single(0,0), process_noice_single(0,1),
                                               process_noice_single(1,0), process_noice_single(1,0), process_noice_single(1,1);
    }

    void RpmEstimator::setMeasurementNoise(double distance, bool is_double_measure)
    {
        double measurement_noise_yaw;
        if(!is_double_measure) // 单装甲板观测
        {
            if(distance < 1.5) // 分段线性，距离越远yaw的测量噪声越大
            {
                measurement_noise_yaw = 0.5;
            }
            else if (distance < 4.5)
            {
                measurement_noise_yaw = 0.75;
            }
            else
            {
                measurement_noise_yaw = 1.0;
            }
            // yaw_1和yaw_2之间有协方差
            yaw_filter->measurement_cov_maxtrix << measurement_noise_yaw, measurement_noise_yaw , 0,
                                                   measurement_noise_yaw, measurement_noise_yaw, 0,
                                                   0, 0, measurement_noise_yaw / 100.0;
        }
        else // 双装甲板观测
        {
            if(distance < 1.5) // 分段线性，距离越远yaw的测量噪声越大
            {
                measurement_noise_yaw = 0.5;
            }
            else if (distance < 4.5)
            {
                measurement_noise_yaw = 1.0;
            }
            else
            {
                measurement_noise_yaw = 1.5;
            }
            yaw_filter->measurement_cov_maxtrix.diagonal() << measurement_noise_yaw, measurement_noise_yaw, measurement_noise_yaw / 100.0;
        }
        
    }

    void RpmEstimator::setTransitionMatrix()
    {
        yaw_filter->transition_matrix << 1, 0, update_time,
                                         0, 1, update_time,
                                         0, 0, 1;
    }

    double RpmEstimator::calculateSingleYaw(Sophus::SE3<double> pose)
    {
        Eigen::Vector3d eular_angles;
        Eigen::Matrix3d R = pose.so3().matrix().inverse();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R_new = svd.matrixU() * svd.matrixV().transpose();
        eular_angles = R_new.eulerAngles(2, 1, 0);
        return eular_angles[0];
    }

    std::pair<double,double> RpmEstimator::calculateDoubleYaw(Sophus::SE3<double> pose_1, Sophus::SE3<double> pose_2)
    {
        Eigen::Vector3d eular_angles_1;
        Eigen::Matrix3d R_1 = pose_1.so3().matrix().inverse();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd_1(R_1, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R_new_1 = svd_1.matrixU() * svd_1.matrixV().transpose();
        eular_angles_1 = R_new_1.eulerAngles(2, 1, 0);
        double yaw_1 = eular_angles_1[0];
        Eigen::Vector3d eular_angles_2;
        Eigen::Matrix3d R_2 = pose_2.so3().matrix().inverse();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd_2(R_2, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R_new_2 = svd_2.matrixU() * svd_2.matrixV().transpose();
        eular_angles_2 = R_new_2.eulerAngles(2, 1, 0);
        double yaw_2 = eular_angles_2[0];
        return std::make_pair(yaw_1,yaw_2);
    }

    Eigen::Vector3d RpmEstimator::makeYawMeasurement(double yaw, int index)
    {
        Eigen::Vector3d yaw_measure_temp;
        switch (index)
        {
        case 0:
            yaw_measure_temp << yaw, yaw + M_PI_2, M_PI_2;
            break;
        case 1:
            yaw_measure_temp << yaw + M_PI_2, yaw + M_PI, M_PI_2;
            break;
        case 2:
            yaw_measure_temp << yaw + M_PI, yaw + 3 * M_PI_2, M_PI_2;
            break;
        case 3:
            yaw_measure_temp << yaw + 3 * M_PI_2, yaw + 2 * M_PI, M_PI_2;
            break;    
        }
        return yaw_measure_temp;
    }

    Eigen::Vector3d RpmEstimator::makeYawMeasurement(double yaw_1, int index_1, double yaw_2, int index_2)
    {
        Eigen::Vector3d yaw_measure_temp;

        if((index_1 - index_2) == 1 || (index_1 - index_2) == -3)
        {
            yaw_1 += M_PI;
            // index_1比index_2大，将角度对齐到0号
            switch (index_2)
            {
            case 0:
                yaw_measure_temp << yaw_2, yaw_1, M_PI_2;
                break;
            case 1:
                yaw_measure_temp << yaw_2 + M_PI_2, yaw_1 + M_PI_2, M_PI_2;
                break;
            case 2:
                yaw_measure_temp << yaw_2 + M_PI, yaw_1 + M_PI, M_PI_2;
                break;
            case 3:
                yaw_measure_temp << yaw_2 + 3 * M_PI_2, yaw_1 + 3 * M_PI_2, M_PI_2;
                break;
            }
        }
        else
        {
            yaw_2 += M_PI;
            // index_2比index_1大, 将角度对齐到0号
            switch (index_1)
            {
            case 0:
                yaw_measure_temp << yaw_1, yaw_2, M_PI_2;
                break;
            case 1:
                yaw_measure_temp << yaw_1 + M_PI_2, yaw_2 + M_PI_2, M_PI_2;
                break;
            case 2:
                yaw_measure_temp << yaw_1 + M_PI, yaw_2 + M_PI, M_PI_2;
                break;
            case 3:
                yaw_measure_temp << yaw_1 + 3 * M_PI_2, yaw_2 + 3 * M_PI_2, M_PI_2;
                break;
            }
        }

        return yaw_measure_temp;
    }

    Eigen::Vector3d RpmEstimator::makeYawContinues(Eigen::Vector3d yaw_measurement)
    {
        double continues_diff = std::round((last_yaw - yaw_measurement[0]) / 2 / CV_PI) * 2 * CV_PI;
        yaw_measurement[0] = yaw_measurement[0] + continues_diff;
        yaw_measurement[1] = yaw_measurement[1] + continues_diff;
        last_yaw = yaw_measurement[0];
        return yaw_measurement;
    }

    int RpmEstimator::getSpinState()
    {
        return this->spin_state;
    }

    double RpmEstimator::getArmorSwitchFrequency(int this_cloest_index)
    {
        if(last_cloest_index == -1)
        {
            spin_state = UNKNOW;
        }
        else
        {
            auto currentTime = std::chrono::system_clock::now();
            // 清除超过1s的老数据
            time_stamp_window.erase(std::remove_if(time_stamp_window.begin(), time_stamp_window.end(), [currentTime](const auto& timePoint) {
                return (currentTime - timePoint) > std::chrono::seconds(3);
            }), time_stamp_window.end());

            if(time_stamp_window.empty())
            {
                spin_state = UNKNOW;
            }

            if(spin_state == UNKNOW)
            {
                if((this_cloest_index - last_cloest_index) == 1 || (this_cloest_index - last_cloest_index) == -3)
                {
                    // 顺时针，将状态设置为此，累计一段时间不切换状态后抛出装甲板切换频率
                    spin_state = CLOCK_WISE;
                    time_stamp_window.push_back(currentTime);
                }
                else if((this_cloest_index - last_cloest_index) == -1 || (this_cloest_index - last_cloest_index) == 3)
                {
                    // 逆时针，将状态设置为此，累计一段时间不切换状态后抛出装甲板切换频率
                    spin_state = COUNTER_CLOCK_WISE;
                    time_stamp_window.push_back(currentTime);
                }
                else if(this_cloest_index == last_cloest_index)
                {
                    spin_state = UNKNOW;
                    time_stamp_window.clear();
                }
            }
            else if(spin_state == CLOCK_WISE)
            {
                if((this_cloest_index - last_cloest_index) == 1 || (this_cloest_index - last_cloest_index) == -3) // 顺时针切换
                {
                    time_stamp_window.push_back(currentTime);
                }
                if((this_cloest_index - last_cloest_index) == -1 || (this_cloest_index - last_cloest_index) == 3) // 出现一次逆时针切换
                {
                    spin_state = UNKNOW;
                    time_stamp_window.clear();
                }
            }
            else if(spin_state == COUNTER_CLOCK_WISE)
            {
                if((this_cloest_index - last_cloest_index) == 1 || (this_cloest_index - last_cloest_index) == -3) // 出现一次逆时针切换
                {
                    spin_state = UNKNOW;
                    time_stamp_window.clear();
                }
                if((this_cloest_index - last_cloest_index) == -1 || (this_cloest_index - last_cloest_index) == 3) // 又一次顺时针切换
                {
                    time_stamp_window.push_back(currentTime);
                }
            }
        }
        last_cloest_index = this_cloest_index;

        int angle_speed_sign;
        if(spin_state != UNKNOW && time_stamp_window.size() > MIN_WINDOW_SIZE)
        {
            if(spin_state == 1)
            {
                COUT("CLOCK_WISE", BLUE);
                angle_speed_sign = 1;
            }
            else if(spin_state == -1)
            {
                COUT("COUNTER_CLOCK_WISE", BLUE);
                angle_speed_sign = -1;
            }

            // 计算最新时间戳和最老时间戳的时间差（毫秒）
            std::chrono::duration<double, std::milli> time_diff = time_stamp_window.back() - time_stamp_window.front();
            double time_diff_in_seconds = time_diff.count() / 1000.0;
            return (double)angle_speed_sign * (time_stamp_window.size() - 1) / time_diff_in_seconds;
        }
        else
        {
            COUT("SPIN STATE UNKNOW", RED);
            return -999.9;
        }
    }

    void RpmEstimator::rebootEstimator(SOLVER::IndexedArmorPoses indexed_poses)
    {
        Eigen::Vector3d yaw_measurement;
        if(indexed_poses.size() == 1) // 只有单装甲板观测
        {
            double single_yaw = indexed_poses.at(0).second.yaw_world;
            yaw_measurement = makeYawMeasurement(single_yaw,indexed_poses.at(0).first);
        }
        else // 有双装甲板观测
        {
            yaw_measurement = makeYawMeasurement(indexed_poses.at(0).second.yaw_world,indexed_poses.at(0).first,indexed_poses.at(1).second.yaw_world,indexed_poses.at(1).first);
        }

        this->yaw_filter->posteriori_state_estimate = yaw_measurement;
        this->yaw_filter->error_cov_post = Eigen::Matrix<double, 3, 3>::Identity();
    }
    
    Eigen::Vector2d RpmEstimator::getDoubleAngle()
    {
        return {this->yaw_filter->posteriori_state_estimate[0], this->yaw_filter->posteriori_state_estimate[1] - M_PI};
    }

    double RpmEstimator::angleFusion(double angle_filter, double angle_statistical)
    {
        std::cout<<"window size:"<<this->time_stamp_window.size()<<std::endl;
        int k = this->time_stamp_window.size();
        // return angle_filter;
        if(angle_statistical == 0.)
        {
            return angle_filter;
        }
        else
        {
            return double(1.0 / (1.0 + k)) * angle_filter + double(k / (1.0 + k)) * angle_statistical;
        }
    }   

} // namespace PREDICTOR


