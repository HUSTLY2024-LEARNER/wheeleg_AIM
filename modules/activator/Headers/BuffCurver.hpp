#pragma once

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "ceres/ceres.h"

namespace ACTIVATOR
{    
    struct SPEED_FITTING_COST
    {
        SPEED_FITTING_COST(double speed, double t) : _speed(speed), _t(t) {}
        // 残差的计算
        template <typename T>
        bool operator()(
            const T *const params, // 模型参数，有3维
            T *residual) const     // 残差
        {
            residual[0] = _speed - (params[0] * ceres::sin(params[1] * _t + params[2]) + 2.090 - params[0]);
            return true;
        }
        const double _speed, _t; // x,y,z数据
    };

    struct LargeBuffFitFunction2
    {
        //传进来角度
        LargeBuffFitFunction2(double speed,double t, double a, double w) : _t(t), _speed(speed), _a(a), _w(w) {}
        // 残差的计算
        template <typename T>
        bool operator()(
            const T *const params, // 模型参数，有4个,a,w,theta ,其中b=2.090-a,可舍去
            T *residual) const     // 残差一维
        {
            residual[0] = _speed - (_a * ceres::sin(_w * _t + params[0]) + 2.090 - _a);
            return true;
        }
        const double _t, _speed, _a, _w; // t，angle
    };
    
    class BuffCurver
    {
    public:
        BuffCurver();
        ~BuffCurver();

        const double *fitLargeBuffSin(const std::vector<float> &speed, const std::vector<float> &t);
        const double *fitLargeBuffSin2(const std::vector<float> &speed, const std::vector<float> &t);
    private:
        double estimate_cos_params[3] = {1.0, 2, 0};
    };
}