#pragma once

#include "BuffCurver.hpp"
#include "BuffAngleKalman.hpp"
#include "detector.hpp"

namespace ACTIVATOR
{
#define CONVERGE_COUNT 1
#define SPEED_FIT_SIZE 200

#define CLOCK_WISE 0
#define COUNTER_CLOCK_WISE 1

    class BuffPredictor
    {
    public:
        BuffPredictor();
        ~BuffPredictor();

        void setPredictMode(int type); // 0 小符； 1 大符

        static double updateDeltaTime(const long& time_stamp)
        {
        static int update_count = 0;
        static long last_time_stamp;
        double average_delta_time;
        if(update_count == 0){
            average_delta_time = (double)10/(double)1000;
            last_time_stamp = time_stamp;
        }else{
            average_delta_time = (double(time_stamp - last_time_stamp))/double(1000);
            last_time_stamp = time_stamp;
        }
        update_count++;
        return average_delta_time;
        }

        void setT(double t){this->delta_time = t;};

        float OnePointPredict(DETECTOR::BLine);
        cv::Point2f OnePointPositonCalc(const cv::Point2f &center, const cv::Point2f &fan_center, const float &predict_time, const float &filte_angle);

    private:
        std::unique_ptr<BuffCurver> buff_curver;
        std::unique_ptr<BuffAngleKalman> buff_angle_kalman;

        float makeAngleContinous();
        DETECTOR::BLine this_aim_fan;
        DETECTOR::BLine last_aim_fan;
        float this_fan_angle;
        float last_fan_angle;
        float last_last_fan_angle = 0.0f;
        long last_time_stamp;
        bool is_get_first_armor;
        float time_base;
        mutable int buff_type = -1;  // 0 小符，1 大符，-1 未定
        float filte_speed = 0;
        bool is_get_sine;  // 是否已经得到拟合参数
        bool is_get_rotation = false;  // 是否计算好了方向
        float rotation = -1; // 旋转方向
        int clockwise_count;

        // 用于正弦拟合
        std::vector<float> buff_angle_set;
        std::vector<float> buff_time_set;
        std::vector<float> speed_set;

        float time_start = 0.0f; // 成功拟合的时间初始
        const double *cos_params; // 拟合参数
        SineFunction *sine_function;

        float filteSpeed(const float &);  // 滤出速度
        void largeBuffFit();
        void judgeRotation(float angle_diff);

        double delta_time;
    };
}
