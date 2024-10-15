#include "LinearEstimator.hpp"

#define PROCESS_NOISE_X_LINEAR 500
#define PROCESS_NOISE_Y_LINEAR 500
#define PROCESS_NOISE_Z_LINEAR 60

namespace PREDICTOR
{
    Eigen::Vector3d translate2xyz(Eigen::Vector3d translate)
    {
        return {translate[0], translate[2], -translate[1]};
    };

    Eigen::Vector3d translate2pyd(Eigen::Vector3d translate)
    {
        Eigen::Vector3d xyz = {translate[0], translate[2], -translate[1]};
        Eigen::Vector3d pyd;
        pyd[2] = xyz.norm();
        pyd[0] = ceres::atan2(xyz[2], sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1])); // pitch
        pyd[1] = ceres::atan2(xyz[0], xyz[1]);
        return pyd;
    }

    LinearEstimator::LinearEstimator()
    {
        this->linear_filter_ca = std::make_unique<ExtendedKalman<double,9,3>>();
        this->linear_filter_cv = std::make_unique<ExtendedKalman<double,6,3>>();
        this->linear_filter_static = std::make_unique<ExtendedKalman<double,3,3>>();
        this->linear_filter_cv_sphere = std::make_unique<ExtendedKalman<double,6,3>>();
    };

    LinearEstimator::~LinearEstimator()
    {

    };

    void LinearEstimator::setT(double t)
    {
        this->update_time = t;
    }

    void LinearEstimator::inputMeasurement(SOLVER::IndexedArmorPoses indexed_poses)
    {
        if(indexed_poses.at(0).first != this->followed_index)
        {
            rebootEstimator(indexed_poses);
        }
        
        setMeasurementNoise(indexed_poses.at(0).second.translate.norm());
        setProcessNoise();
        setTransitionMatrix();

        linear_filter_ca->predict(linear_ca_state_2_measure, translate2xyz(indexed_poses.at(0).second.translate));
        Eigen::Vector<double, 9> linear_state_ca = linear_filter_ca->update();

        linear_filter_cv->predict(linear_cv_state_2_measure, translate2xyz(indexed_poses.at(0).second.translate));
        Eigen::Vector<double, 6> linear_state_cv = linear_filter_cv->update();

        linear_filter_static->predict(linear_static_state_2_measure, translate2xyz(indexed_poses.at(0).second.translate));
        Eigen::Vector<double, 3> linear_state_static = linear_filter_static->update();

        linear_filter_cv_sphere->predict(linear_cv_state_2_pyd_measure, translate2pyd(indexed_poses.at(0).second.translate));
        Eigen::Vector<double, 6> linear_state_cv_sphere = linear_filter_cv_sphere->update();

        this->filtered_position_ca = {linear_state_ca[0], linear_state_ca[3], linear_state_ca[6]};
        this->filtered_speed_ca = {linear_state_ca[1], linear_state_ca[4], linear_state_ca[7]};
        this->filtered_acc_ca = {linear_state_ca[2], linear_state_ca[5], linear_state_ca[8]};

        this->filtered_position_cv = {linear_state_cv[0], linear_state_cv[2], linear_state_cv[4]};
        this->filtered_speed_cv = {linear_state_cv[1], linear_state_cv[3], linear_state_cv[5]};

        this->filtered_position_static = {linear_state_static[0], linear_state_static[1], linear_state_static[2]};

        this->filtered_position_cv_sphere = {linear_state_cv_sphere[0], linear_state_cv_sphere[2], linear_state_cv_sphere[4]};
        this->filtered_speed_cv_sphere = {linear_state_cv_sphere[1], linear_state_cv_sphere[3], linear_state_cv_sphere[5]};

        getDifferentChiSquardValue();
    };

    void LinearEstimator::setTransitionMatrix()
    {
        Eigen::Matrix3d transition_CA;
        transition_CA << 1, update_time, 0.5 * update_time * update_time, 
                        0, 1, update_time,
                        0, 0, 1;
        
        Eigen::Matrix2d transition_CV;
        transition_CV << 1, update_time, 
                        0, 1;

        for(int i = 0; i < 3; i++)
        {
            linear_filter_ca->transition_matrix.block<3, 3>(i * 3, i * 3) = transition_CA;
        }

        for(int i = 0; i < 3; i++)
        {
            linear_filter_cv->transition_matrix.block<2, 2>(i * 2, i * 2) = transition_CV;
        }

        for(int i = 0; i < 3; i++)
        {
            linear_filter_cv_sphere->transition_matrix.block<2, 2>(i * 2, i * 2) = transition_CV;
        }


        linear_filter_static->transition_matrix << 1, 0, 0,
                                                    0, 1, 0,
                                                    0, 0, 1; 
    }

    void LinearEstimator::setProcessNoise()
    {
        Eigen::Matrix3d process_noise_matrix_CA;
        process_noise_matrix_CA << std::pow(update_time, 5) / 20.0, std::pow(update_time, 4) / 8.0, std::pow(update_time, 3) / 6.0,
                                    std::pow(update_time, 4) / 8.0, std::pow(update_time, 3) / 3.0, std::pow(update_time, 2) /2.0,
                                    std::pow(update_time, 3) / 6.0, std::pow(update_time, 2) /2.0, update_time;
        
        Eigen::Matrix<double, 2, 1> process_noice_vec_cv;
        process_noice_vec_cv << 0.5 * update_time * update_time, update_time;
        Eigen::Matrix2d process_noise_matrix_CV = process_noice_vec_cv * process_noice_vec_cv.transpose();

        this->linear_filter_ca->process_noise_cov.block<3, 3>(0, 0) = PROCESS_NOISE_X_LINEAR * process_noise_matrix_CA;
        this->linear_filter_ca->process_noise_cov.block<3, 3>(3, 3) = PROCESS_NOISE_Y_LINEAR * process_noise_matrix_CA;
        this->linear_filter_ca->process_noise_cov.block<3, 3>(6, 6) = PROCESS_NOISE_Z_LINEAR * process_noise_matrix_CA;

        this->linear_filter_cv->process_noise_cov.block<2, 2>(0, 0) = PROCESS_NOISE_X_LINEAR * process_noise_matrix_CV;
        this->linear_filter_cv->process_noise_cov.block<2, 2>(2, 2) = PROCESS_NOISE_Y_LINEAR * process_noise_matrix_CV;
        this->linear_filter_cv->process_noise_cov.block<2, 2>(4, 4) = PROCESS_NOISE_Z_LINEAR * process_noise_matrix_CV;

        this->linear_filter_cv_sphere->process_noise_cov.block<2, 2>(0, 0) = PROCESS_NOISE_X_LINEAR * process_noise_matrix_CV;
        this->linear_filter_cv_sphere->process_noise_cov.block<2, 2>(2, 2) = PROCESS_NOISE_Y_LINEAR * process_noise_matrix_CV;
        this->linear_filter_cv_sphere->process_noise_cov.block<2, 2>(4, 4) = PROCESS_NOISE_Z_LINEAR * process_noise_matrix_CV;

        this->linear_filter_static->process_noise_cov.diagonal() << PROCESS_NOISE_X_LINEAR * update_time, PROCESS_NOISE_Y_LINEAR * update_time, PROCESS_NOISE_Z_LINEAR * update_time;
    }

    void LinearEstimator::setMeasurementNoise(double distance)
    {
        double measure_noise_x, measure_noise_y, measure_noise_z;

        if(distance < 1.5)
        {
            measure_noise_x = 0.0005;
            measure_noise_y = 0.0005;
            measure_noise_z = 0.0005;
        }
        else if(distance < 4.5)
        {
            measure_noise_x = 0.00075;
            measure_noise_y = 0.00075;
            measure_noise_z = 0.00075;
        }
        else
        {
            measure_noise_x = 0.001;
            measure_noise_y = 0.001;
            measure_noise_z = 0.001;
        }

        linear_filter_ca->measurement_cov_maxtrix.diagonal() << measure_noise_x, measure_noise_y, measure_noise_z;
        linear_filter_cv->measurement_cov_maxtrix.diagonal() << measure_noise_x, measure_noise_y, measure_noise_z;
        linear_filter_static->measurement_cov_maxtrix.diagonal() << measure_noise_x, measure_noise_y, measure_noise_z;

        double measure_noise_pitch, measure_noise_yaw, measure_noise_distance;
        measure_noise_pitch = 0.0001;
        measure_noise_yaw = 0.0001;
        if (distance < 1.5) // 统计方法计算，分段线性
        {
            measure_noise_distance = pow(distance * 0.01, 2);
        }
        else if (distance < 4.5)
        {
            measure_noise_distance = pow(0.015 + 0.058 * (distance - 1.5), 2);
        }
        else
        {
            measure_noise_distance = pow(0.189 + 0.03 * (distance - 4.5), 2);
        }

        linear_filter_cv_sphere->measurement_cov_maxtrix.diagonal() << measure_noise_pitch, measure_noise_yaw, measure_noise_distance;
    }

    void LinearEstimator::rebootEstimator(SOLVER::IndexedArmorPoses indexed_armor_poses)
    {
        followed_index = indexed_armor_poses.at(0).first;
        Eigen::Vector3d xyz = translate2xyz(indexed_armor_poses.at(0).second.translate);

        this->linear_filter_ca->error_cov_post = Eigen::Matrix<double, 9, 9>::Identity();
        this->linear_filter_ca->posteriori_state_estimate = {xyz[0], 0, 0.5, xyz[1], 0, 0.5, xyz[2], 0, 0.1};

        this->linear_filter_cv->error_cov_post = Eigen::Matrix<double, 6, 6>::Identity();
        this->linear_filter_cv->posteriori_state_estimate = {xyz[0], 0, xyz[1], 0, xyz[2], 0};

        this->linear_filter_cv_sphere->error_cov_post = Eigen::Matrix<double, 6, 6>::Identity();
        this->linear_filter_cv_sphere->posteriori_state_estimate = {xyz[0], 0, xyz[1], 0, xyz[2], 0};

        this->linear_filter_static->error_cov_post = Eigen::Matrix<double, 3, 3>::Identity();
        this->linear_filter_static->posteriori_state_estimate = {xyz[0], xyz[1], xyz[2]};
    };

    void LinearEstimator::getDifferentChiSquardValue()
    {
        this->chi_squard_ca = linear_filter_ca->residual.transpose() * (linear_filter_ca->measurement_matrix * linear_filter_ca->process_noise_cov * linear_filter_ca->measurement_matrix.transpose() + linear_filter_ca->measurement_noise_cov).inverse() * linear_filter_ca->residual;
        this->chi_squard_cv = linear_filter_cv->residual.transpose() * (linear_filter_cv->measurement_matrix * linear_filter_cv->process_noise_cov * linear_filter_cv->measurement_matrix.transpose() + linear_filter_cv->measurement_noise_cov).inverse() * linear_filter_cv->residual;
        this->chi_squard_static = linear_filter_static->residual.transpose() * (linear_filter_static->measurement_matrix * linear_filter_static->process_noise_cov * linear_filter_static->measurement_matrix.transpose() + linear_filter_static->measurement_noise_cov).inverse() * linear_filter_static->residual;

        // std::cout<<"ca: "<<this->chi_squard_ca<<std::endl;
        // std::cout<<"cv: "<<this->chi_squard_cv<<std::endl;
        // std::cout<<"static: "<<this->chi_squard_static<<std::endl;

        // std::cout<<"ca re: "<<linear_filter_ca->residual<<std::endl;
        // std::cout<<"cv re: "<<linear_filter_cv->residual<<std::endl;
        // std::cout<<"static re: "<<linear_filter_static->residual<<std::endl;
    }

    Eigen::Vector3d LinearEstimator::linearFusion(Eigen::Vector3d ca_predicted_position, Eigen::Vector3d cv_predicted_position, Eigen::Vector3d static_predicted_position)
    {
        double fusion_param_ca = 1. / this->chi_squard_ca;
        double fusion_param_cv = 1. / this->chi_squard_cv;
        double fusion_param_static = 1. / this->chi_squard_static;
        double fusion_param_all = fusion_param_ca + fusion_param_cv + fusion_param_static;
        double fusion_percent_ca = fusion_param_ca / fusion_param_all;
        double fusion_percent_cv = fusion_param_cv / fusion_param_all;
        double fusion_percent_static = fusion_param_static / fusion_param_all;

        COUT("CA: "<<(int)(fusion_percent_ca*100)<<"/% CV: "<<(int)(fusion_percent_cv*100)<<"/% STATIC: "<<(int)(fusion_percent_static*100)<<"/%", CYAN);

        return (fusion_param_ca * ca_predicted_position + fusion_param_cv * cv_predicted_position + fusion_param_static * static_predicted_position) / fusion_param_all;
    }

    std::vector<Eigen::Vector3d> LinearEstimator::getMessageCA()
    {
        return {this->filtered_position_ca, this->filtered_speed_ca, this->filtered_acc_ca};
    }

    std::vector<Eigen::Vector3d> LinearEstimator::getMessageCV()
    {
        return {this->filtered_position_cv, this->filtered_speed_cv};
    }

    Eigen::Vector3d LinearEstimator::getMessageStatic()
    {
        return this->filtered_position_static;
    }

    std::vector<Eigen::Vector3d> LinearEstimator::getMessageSphereCV()
    {
        return {this->filtered_position_cv_sphere, this->filtered_speed_cv_sphere};
    }
}