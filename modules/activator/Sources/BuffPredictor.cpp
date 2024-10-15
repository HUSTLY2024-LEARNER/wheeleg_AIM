#include "BuffPredictor.hpp"
#include "utils.h"

using namespace LY_UTILS;

#define DEFAULT_DELTA_TIME 10

namespace ACTIVATOR
{
    BuffPredictor::BuffPredictor()
    {
        buff_curver = std::make_unique<BuffCurver>();
        buff_angle_kalman = std::make_unique<BuffAngleKalman>();
        sine_function = new SineFunction();
    }

    BuffPredictor::~BuffPredictor()
    {
        delete sine_function;
    }

    float BuffPredictor::makeAngleContinous()
    {
        this_fan_angle = this_aim_fan.angle + round((last_fan_angle - this_aim_fan.angle) / 2 / CV_PI) * 2 * CV_PI;
        return this_fan_angle;
    }

    float BuffPredictor::OnePointPredict(DETECTOR::BLine aim_fan)
    {
        this_aim_fan = aim_fan;

        float delta_t = this->delta_time;
	// std::cout<<"delta_t:"<<delta_t<<std::endl;

        if(!is_get_first_armor)
        {
            is_get_first_armor = true;
            time_base = 0.0f;
            last_fan_angle = aim_fan.angle;
        }
        else
        {
            time_base += delta_t;
        }

        makeAngleContinous();

        float filte_angle;

        if(buff_type == 0) // 小符
        {
            filte_angle = this_fan_angle;
            COUT("SMALL BUFF", MAGENTA);
        }
        else
        {
            filte_speed = filteSpeed(delta_t);
            COUT("BIG BUFF", MAGENTA);
        }

        largeBuffFit();
        if(is_get_sine)
        {
            filte_angle = this_fan_angle;
        }
        else
        {
            filte_angle = 999.0f;
        }

        if(!is_get_rotation)
        {
            judgeRotation(this_fan_angle - last_fan_angle); // 判断旋转方向
        }

        if (rotation == 0)
        {
            COUT("ROTATION: CLOCK_WISE",GREEN);
        }
        else if (rotation == 1)
        {
            COUT("ROTATION: COUNTER_CLOCK_WISE",GREEN);
        }
        last_last_fan_angle = last_fan_angle;
        last_fan_angle = this_fan_angle; // 更新角度
        last_aim_fan = this_aim_fan;

        return filte_angle;
    }

    float BuffPredictor::filteSpeed(const float &delta_t)
    {
        static float last_angle_temp = 0.0;
        float this_angle_temp = this_fan_angle - round((this_fan_angle - last_angle_temp) / 2 / M_PI * 5) * 2 * M_PI / 5;
        last_angle_temp = this_angle_temp;

        buff_angle_kalman->runKalman(this_angle_temp, delta_t);
        return buff_angle_kalman->getSpeed();
    }

    void BuffPredictor::largeBuffFit()
    {
        static int count = 0;
        if (count >= CONVERGE_COUNT)
        {
            speed_set.push_back(filte_speed);
            buff_time_set.push_back(time_base);
        }

        count++;

        if (speed_set.size() < SPEED_FIT_SIZE || !is_get_rotation)
        {
            return;
        }

        if (rotation == CLOCK_WISE) // 转为正速度
        {
            for (int i = 0; i < speed_set.size(); i++)
            {
                speed_set[i] = -speed_set[i];
            }
        }

        time_start = buff_time_set[0];
        for (int i = 0; i < buff_time_set.size(); i++)
        {
            buff_time_set[i] -= time_start;
        }

        if (!is_get_sine)
        {
            cos_params = buff_curver->fitLargeBuffSin(speed_set, buff_time_set);
        }
        else
        {
            cos_params = buff_curver->fitLargeBuffSin2(speed_set, buff_time_set);
        }

        sine_function->setParams(cos_params);
        is_get_sine = true;
        count = 0; // 清空缓冲

        std::vector<float>().swap(speed_set);
        std::vector<float>().swap(buff_time_set);
    }

    void BuffPredictor::setPredictMode(int type)
    {
        buff_type = type;
    }

    void BuffPredictor::judgeRotation(float angle_diff)
    {
        if (is_get_rotation)
        {
            return;
        }
        angle_diff > 0 ? clockwise_count++ : clockwise_count--;
        if (clockwise_count > 30)
        {
            is_get_rotation = true;
            rotation = COUNTER_CLOCK_WISE; // 逆时针
        }
        else if (clockwise_count < -30)
        {
            is_get_rotation = true;
            rotation = CLOCK_WISE; // 顺时针
        }
    }

    cv::Point2f BuffPredictor::OnePointPositonCalc(const cv::Point2f &center, const cv::Point2f &armor_center, const float &predict_time, const float &filte_angle)
    {
        cv::Point2f center_to_armor = center - armor_center;
        float radius = sqrt(center_to_armor.x * center_to_armor.x + center_to_armor.y * center_to_armor.y);

        if (buff_type == 0) // SMALL_BUFF
        {
            if (rotation == CLOCK_WISE) // 顺时针
            {
                return center + cv::Point2f(radius * cos(filte_angle - 1.047 * predict_time), -radius * sin(filte_angle - 1.047 * predict_time));
            }
            else if (rotation == COUNTER_CLOCK_WISE)
            {
                return center + cv::Point2f(radius * cos(filte_angle + 1.047 * predict_time), -radius * sin(filte_angle + 1.047 * predict_time));
            }
            else
            {
                return cv::Point2f(-1, -1);
            }
        }
        else // 大符模式
        {
            if (fabs(filte_angle - 999) < 1e-4) // 说明未拟合到角度
            {
                return cv::Point2f(-1, -1);
            }
            else
            {
                float aim_angle = filte_angle + (sine_function->predict(time_base + predict_time - time_start) - sine_function->predict(time_base - time_start)) * pow(-1, rotation + 1);
                return center + cv::Point2f(radius * cos(aim_angle), -radius * sin(aim_angle));
            }
        }
    }

}
