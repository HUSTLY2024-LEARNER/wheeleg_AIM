#pragma once 

#include"driver.hpp"

namespace PREDICTOR
{   
    // 过程噪声设小
    class PitchYawFilter
    {
    public:
        PitchYawFilter()
        {
            A << 1, 0,
                0, 1;

            H << 1, 0,
                0, 1;

            Q << 1, 0,
                0, 1;

            R << 1, 0,
                0, 1;

            P << 1, 0,
                0, 1;

            x << 0, 0;
            
            residual << 0, 0;
        }
    private:
        Eigen::Matrix2d A;  // 状态转移矩阵
        Eigen::Matrix2d H;  // 观测矩阵

        Eigen::Matrix2d Q;  // 过程噪声协方差
        Eigen::Matrix2d R;  // 观测噪声协方差
        Eigen::Matrix2d P;  // 估计误差协方差

        Eigen::Vector2d x;  // 状态向量
        Eigen::Vector2d residual;  //残差向量
    };

    class TimedGimbalTask
    {
    public:
        void inputSendMessage();
        DRIVER::SerialWriteData getPredictSendMessage();
    private:
        
    
    };

} // namespace PREDICTOR
