#pragma once

#include "BLines.h"
#include "openvino/openvino.hpp"

namespace DETECTOR
{   
    class BuffBBox
    {
        //params
        static constexpr int INPUT_W = 416;   // Width of input
        static constexpr int INPUT_H = 416;   // Height of input
        static constexpr int NUM_CLASSES = 8; // Number of classes
        static constexpr int NUM_COLORS = 8;  // Number of color
        static constexpr int TOPK = 128;      // TopK
        static constexpr float NMS_THRESH = 0.3;
        static constexpr float BBOX_CONF_THRESH = 0.6;
        static constexpr float MERGE_CONF_ERROR = 0.15;
        static constexpr float MERGE_MIN_IOU = 0.9;

    public:
        explicit BuffBBox(const std::string &model_file);

        ~BuffBBox();

        BLines operator()(const cv::Mat &img);
    
    private:
        ov::Core core;
        std::shared_ptr<ov::Model> model; // 网络
        ov::CompiledModel compiled_model; // 可执行网络
        ov::InferRequest infer_request;   // 推理请求
        ov::Tensor input_tensor;

        void initModel(const std::string &model_file);
    };
} //namespace DETECTOR