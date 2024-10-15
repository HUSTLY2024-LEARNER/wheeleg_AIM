#include "BuffCurver.hpp"

namespace ACTIVATOR
{
    BuffCurver::BuffCurver(/* args */)
    {
    }

    BuffCurver::~BuffCurver()
    {
    }

    const double *BuffCurver::fitLargeBuffSin(const std::vector<float> &speed, const std::vector<float> &t)
    {
        // 新大符
        ceres::Problem problem;
        problem.AddParameterBlock(estimate_cos_params, 3); // 添加优化参数

        // //设置a范围
        problem.SetParameterLowerBound(estimate_cos_params, 0, 0.780);
        problem.SetParameterUpperBound(estimate_cos_params, 0, 1.045);

        // // //设置w范围
        problem.SetParameterLowerBound(estimate_cos_params, 1, 1.884);
        problem.SetParameterUpperBound(estimate_cos_params, 1, 2.000);

        for (int i = 0; i < t.size(); i++)
        {
            problem.AddResidualBlock(
                // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<SPEED_FITTING_COST, 1, 3>(
                    new SPEED_FITTING_COST(speed[i], t[i])),
                new ceres::CauchyLoss(0.5),
                estimate_cos_params // 待估计参数
            );
        }
        // 配置求解器
        ceres::Solver::Options options;               // 这里有很多配置项可以填
        options.linear_solver_type = ceres::DENSE_QR; // 增量方程如何求解
        options.minimizer_progress_to_stdout = true;  // 输出到cout
        options.max_num_iterations = 50;              // 迭代次数
        options.num_threads = 1;

        ceres::Solver::Summary summary;            // 优化信息
        ceres::Solve(options, &problem, &summary); // 开始优化
        return estimate_cos_params;
    }

    const double *BuffCurver::fitLargeBuffSin2(const std::vector<float> &speed, const std::vector<float> &t)
    {
        // 新大符
        ceres::Problem problem;
        double theta = 0;

        for (int i = 0; i < t.size(); i++)
        {
            problem.AddResidualBlock(
                // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<LargeBuffFitFunction2, 1, 1>(
                    new LargeBuffFitFunction2(speed[i], t[i], estimate_cos_params[0], estimate_cos_params[1])),
                new ceres::CauchyLoss(0.5),
                &theta // 待估计参数
            );
        }
        // 配置求解器
        ceres::Solver::Options options;               // 这里有很多配置项可以填
        options.linear_solver_type = ceres::DENSE_QR; // 增量方程如何求解
        options.minimizer_progress_to_stdout = true;  // 输出到cout
        options.max_num_iterations = 50;              // 迭代次数
        options.num_threads = 1;

        ceres::Solver::Summary summary;            // 优化信息
        ceres::Solve(options, &problem, &summary); // 开始优化

        estimate_cos_params[2] = theta;
        return estimate_cos_params;
    }
}