#pragma once

#include "BLines.h"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "openvino/openvino.hpp"
#include "ArmorOneStage.hpp"

namespace DETECTOR
{   
    struct BuffArmor
    {
        BuffArmor()
        {
            is_useful = false;
            corners = std::vector<cv::Point2f>(5);
        }

        std::vector<cv::Point2f> corners; // 存放最终的5点
        cv::Point2f apex[5];
        std::vector<cv::Point2f> pts; // 用来进行防抖的处理
        cv::Rect_<float> rect;        // 矩形框（nms处理要用到）
        int cls;                      // 类别：0——已激活，1——未激活，2——R标
        int color;                    // 颜色
        float prob;                   // 可信度

        cv::RotatedRect armor_rect;
        float armor_ratio;
        cv::Point2f direction_vec; // 中心指向装甲板的向量
        float angle;
        Eigen::Vector3d world_pose;
        bool is_useful;
        cv::Point2f buff_center;

        std::chrono::_V2::steady_clock::time_point time_stamp;
    };

    class BuffFivePoints
    {
    public:
        explicit BuffFivePoints(const std::string &model_file);

        ~BuffFivePoints();

        BLine operator()(const cv::Mat &img);

    private:
        void initModel(const std::string &model_file);

        ov::Core core;
        std::shared_ptr<ov::Model> model; // 网络
        ov::CompiledModel compiled_model; // 可执行网络
        ov::InferRequest infer_request;   // 推理请求
        ov::Tensor input_tensor;

        Eigen::Matrix<float, 3, 3> transfrom_matrix;
        BuffArmor aim_target;
    };
} //namespace DETECTOR