#pragma once

#include "BBoxes.h"
#include "openvino/openvino.hpp"
#include "NumberClassifier.hpp"
#include <Eigen/Eigen>
#include <utils.h>

namespace DETECTOR
{    
    class ArmorOneStage
    {
    public:
        explicit ArmorOneStage(const std::string &model_file);

        ~ArmorOneStage();

        BBoxes operator()(const cv::Mat &img);
        cv::Mat getProposalPic();
        void setColorFlag(int flag_)
        {
            if(flag_ == 0) // enemy_red
            {
                this->color_flag = 1;
                COUT("ENEMY RED", RED);
            }
            else
            {
                this->color_flag = 0;
                COUT("ENEMY BLUE", BLUE);
            }
        };
    private:
        std::unique_ptr<NumberClassifier> number_classifier;

        ov::Core core;
        std::shared_ptr<ov::Model> model; // 网络
        ov::CompiledModel compiled_model; // 可执行网络
        ov::InferRequest infer_request;   // 推理请求
        ov::Tensor input_tensor;

        Eigen::Matrix<float, 3, 3> transfrom_matrix;

        void initModel(const std::string &model_file);
        cv::Mat drawProposals;

        //blue: 0, red: 1, gray: 2, purple: 3
        int color_flag = -1;
    };

    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };
} //namespace DETECTOR