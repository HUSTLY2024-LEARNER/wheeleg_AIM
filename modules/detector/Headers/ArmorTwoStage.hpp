#pragma once

#include "BBoxes.h"
#include "LightBarFinder.hpp"
#include "NumberClassifier.hpp"
#include <opencv2/opencv.hpp>
#include "openvino/openvino.hpp"

namespace DETECTOR
{
    class ArmorTwoStage
    {
    public:
        explicit ArmorTwoStage(const std::string&model_file, const std::string &color);

        ~ArmorTwoStage();

        BBoxes operator()(const cv::Mat &img);
    
    private:
        std::unique_ptr<LightBarFinder> lightbar_finder;
        std::unique_ptr<NumberClassifier> number_classifier;
        
        ov::Core ie;
        ov::CompiledModel compiled_model;
        ov::InferRequest infer_request;
        ov::Tensor input_tensor;
        ov::Tensor output_tensor;
        ov::Shape input_shape;
        ov::Shape output_shape;

        float score_threshold = 0.45;
        float nms_threshold = 0.5;
        
        double p_width = 1.2; // 将模型得到的ROI的长宽适当扩大 p 倍
        double p_height = 1.5;

        cv::Mat letterbox(const cv::Mat& source);
        void initModel(const std::string &model_file);
        cv::Rect getROI(cv::Mat img, int x1, int x2 ,int y1, int y2);

        int color_flag;
    };
} //namespace DETECTOR