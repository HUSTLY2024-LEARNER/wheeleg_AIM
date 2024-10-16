#include "BuffAngleKalman.hpp"

namespace ACTIVATOR
{
    BuffAngleKalman::BuffAngleKalman()
    {
        is_kalman_init = false;
        kalman_filter = new ExtendedKalman<double, 2, 1>();
        posteriori_angle = 0;
        posteriori_speed = 0;
    }

    BuffAngleKalman::~BuffAngleKalman()
    {
    }

    void BuffAngleKalman::rebootKalman(const double &new_angle)
    {
        kalman_filter->posteriori_state_estimate[0] = new_angle;
        kalman_filter->posteriori_state_estimate[1] = 0;

        kalman_filter->error_cov_post = Eigen::Matrix2d::Identity();
    }

    void BuffAngleKalman::setUpdateTime(const double &delta_t)
    {
        if (fabs(delta_t) < 1e-4) //防止时间差为0
        {
            update_time = 8.0 / 1000.0;
        }
        else
        {
            update_time = delta_t / 1000.0;
        }
    }

    double BuffAngleKalman::runKalman(const double &new_armor_angle, const double &delta_t) //量测有效更新
    {
        if (!is_kalman_init)
        {
            //set signal values
            is_kalman_init = true;

            //reset kalman
            rebootKalman(new_armor_angle);
            setMeasurementNoise();

            //return values
            return new_armor_angle;
        }
        else
        {
            //set update time
            setUpdateTime(delta_t);

            //update transition matrix
            setTransitionMatrix();

            setProcessNoise();
            setMeasurementNoise();

            return correct(new_armor_angle);
        }
    }
    void BuffAngleKalman::setTransitionMatrix()
    {
        Eigen::Matrix2d transition;
        transition << 1, update_time,
            0, 1;

        kalman_filter->transition_matrix = transition;
    }
    double BuffAngleKalman::correct(const double &armor_pose)
    {
        Eigen::Matrix<double, 1, 1> pose;
        pose << armor_pose;
        kalman_filter->predict(buff_state_to_measure, pose); //量测有效更新
        kalman_filter->update();

        posteriori_angle = kalman_filter->posteriori_state_estimate[0];
        posteriori_speed = kalman_filter->posteriori_state_estimate[1];
        return posteriori_angle;
    }
    void BuffAngleKalman::setMeasurementNoise()
    {
        double measurement_noise_angle = 0.01; //待测

        kalman_filter->measurement_noise_cov.diagonal() << measurement_noise_angle;
    }
    void BuffAngleKalman::setProcessNoise()
    {
        Eigen::Matrix<double, 2, 1> process_noice_vec;
        process_noice_vec << 0.5 * update_time * update_time, update_time;

        double angle_process_noise = 1;
        kalman_filter->process_noise_cov = angle_process_noise * process_noice_vec * process_noice_vec.transpose();
    }
    double BuffAngleKalman::predict(const double &predict_t)
    {
        return posteriori_angle + posteriori_speed * predict_t;
    }
}