#include "BuffFivePoints.hpp"
namespace DETECTOR
{
        /**
     * @brief Define names based depends on Unicode path support
     */
    static constexpr int INPUT_W = 640; // Width of input
    static constexpr int INPUT_H = 640; // Height of input
    // static constexpr int INPUT_W = 416;        // Width of input
    // static constexpr int INPUT_H = 416;        // Height of input
    static constexpr int NUM_CLASSES = 2; // Number of classes
    static constexpr int NUM_COLORS = 2;  // Number of color

    static constexpr int TOPK = 128; // TopK
    static constexpr float NMS_THRESH = 0.05;
    static constexpr float BBOX_CONF_THRESH = 0.20;
    static constexpr float MERGE_CONF_ERROR = 0.15;
    static constexpr float MERGE_MIN_IOU = 0.2;

    static inline int argmax(const float *ptr, int len)
    {
        int max_arg = 0;
        for (int i = 1; i < len; i++)
        {
            if (ptr[i] > ptr[max_arg])
                max_arg = i;
        }
        return max_arg;
    }

    /**
     * @brief Resize the image using letterbox
     * @param img Image before resize
     * @param transform_matrix Transform Matrix of Resize
     * @return Image after resize
     */
    inline cv::Mat scaledResize(const cv::Mat &img, Eigen::Matrix<float, 3, 3> &transform_matrix)
    {
        float r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
        int unpad_w = r * img.cols;
        int unpad_h = r * img.rows;

        int dw = INPUT_W - unpad_w;
        int dh = INPUT_H - unpad_h;

        dw /= 2;
        dh /= 2;

        transform_matrix << 1.0 / r, 0, -dw / r,
            0, 1.0 / r, -dh / r,
            0, 0, 1;

        cv::Mat re;
        cv::resize(img, re, cv::Size(unpad_w, unpad_h));
        cv::Mat out;
        cv::copyMakeBorder(re, out, dh, dh, dw, dw, cv::BORDER_CONSTANT);

        return out;
    }

    /**
     * @brief Generate grids and stride.
     * @param target_w Width of input.
     * @param target_h Height of input.
     * @param strides A vector of stride.
     * @param grid_strides Grid stride generated in this function.
     */
    static void generate_grids_and_stride(const int target_w, const int target_h,
                                          std::vector<int> &strides, std::vector<GridAndStride> &grid_strides)
    {
        for (auto stride : strides)
        {
            int num_grid_w = target_w / stride;
            int num_grid_h = target_h / stride;
            for (int g1 = 0; g1 < num_grid_h; g1++)
            {
                for (int g0 = 0; g0 < num_grid_w; g0++)
                {
                    GridAndStride grid_stride = {g0, g1, stride};
                    grid_strides.emplace_back(grid_stride);
                    //
                }
            }
        }
    }

    /**
     * @brief Generate Proposal
     * @param grid_strides Grid strides
     * @param feat_ptr Original predition result.
     * @param prob_threshold Confidence Threshold.
     * @param objects Objects proposed.
     */
    static void generateYoloxProposals(std::vector<GridAndStride> grid_strides, const float *feat_ptr,
                                       Eigen::Matrix<float, 3, 3> &transform_matrix, float prob_threshold,
                                       std::vector<BuffArmor> &objects)
    {
        const int num_anchors = grid_strides.size();

        // Travel all the anchors
        for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
        {
            const int grid0 = grid_strides[anchor_idx].grid0;
            const int grid1 = grid_strides[anchor_idx].grid1;
            const int stride = grid_strides[anchor_idx].stride;

            const int basic_pos = anchor_idx * (11 + NUM_COLORS + NUM_CLASSES);

            // yolox/models/yolo_head.py decode logic
            //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
            //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
            float x_1 = (feat_ptr[basic_pos + 0] + grid0) * stride;
            float y_1 = (feat_ptr[basic_pos + 1] + grid1) * stride;
            float x_2 = (feat_ptr[basic_pos + 2] + grid0) * stride;
            float y_2 = (feat_ptr[basic_pos + 3] + grid1) * stride;
            float x_3 = (feat_ptr[basic_pos + 4] + grid0) * stride;
            float y_3 = (feat_ptr[basic_pos + 5] + grid1) * stride;
            float x_4 = (feat_ptr[basic_pos + 6] + grid0) * stride;
            float y_4 = (feat_ptr[basic_pos + 7] + grid1) * stride;
            float x_5 = (feat_ptr[basic_pos + 8] + grid0) * stride;
            float y_5 = (feat_ptr[basic_pos + 9] + grid1) * stride;

            int box_color = argmax(feat_ptr + basic_pos + 11, NUM_COLORS);
            int box_class = argmax(feat_ptr + basic_pos + 11 + NUM_COLORS, NUM_CLASSES);

            float box_objectness = (feat_ptr[basic_pos + 10]);

            // float color_conf = (feat_ptr[basic_pos + 11 + box_color]);
            // float cls_conf = (feat_ptr[basic_pos + 11 + NUM_COLORS + box_class]);
            // float box_prob = (box_objectness + cls_conf + color_conf) / 3.0;
            float box_prob = box_objectness;

            if (box_prob >= prob_threshold)
            {
                BuffArmor obj;

                Eigen::Matrix<float, 3, 5> apex_norm;
                Eigen::Matrix<float, 3, 5> apex_dst;
                apex_norm << x_1, x_2, x_3, x_4, x_5,
                    y_1, y_2, y_3, y_4, y_5,
                    1, 1, 1, 1, 1;
                apex_dst = transform_matrix * apex_norm;
                for (int i = 0; i < 5; i++)
                {
                    obj.apex[i] = cv::Point2f(apex_dst(0, i), apex_dst(1, i));
                    obj.pts.push_back(obj.apex[i]);
                }
                std::vector<cv::Point2f> tmp(obj.apex, obj.apex + 5);
                obj.rect = cv::boundingRect(tmp);

                obj.cls = box_class;
                obj.color = box_color;
                obj.prob = box_prob;
                objects.push_back(obj);
            }

        } // point anchor loop
    }

    /**
     * @brief Calculate intersection area between two objects.
     * @param a Object a.
     * @param b Object b.
     * @return Area of intersection.
     */
    static inline float intersection_area(const BuffArmor &a, const BuffArmor &b)
    {
        cv::Rect_<float> inter = a.rect & b.rect;
        return inter.area();
    }

    static void qsort_descent_inplace(std::vector<BuffArmor> &faceobjects, int left, int right)
    {
        int i = left;
        int j = right;
        float p = faceobjects[(left + right) / 2].prob;

        while (i <= j)
        {
            while (faceobjects[i].prob > p)
                i++;

            while (faceobjects[j].prob < p)
                j--;

            if (i <= j)
            {
                // swap
                std::swap(faceobjects[i], faceobjects[j]);

                i++;
                j--;
            }
        }

        if (left < j)
            qsort_descent_inplace(faceobjects, left, j);
        if (i < right)
            qsort_descent_inplace(faceobjects, i, right);
    }

    static void qsort_descent_inplace(std::vector<BuffArmor> &objects)
    {
        if (objects.empty())
            return;

        qsort_descent_inplace(objects, 0, objects.size() - 1);
    }

    static void nms_sorted_bboxes(std::vector<BuffArmor> &faceobjects, std::vector<int> &picked,
                                  float nms_threshold)
    {
        picked.clear();
        const int n = faceobjects.size();
        // std::cout << "np:" << n << std::endl;

        std::vector<float> areas(n);
        for (int i = 0; i < n; i++)
        {
            std::vector<cv::Point2f> object_apex_tmp(faceobjects[i].apex, faceobjects[i].apex + 5);
            // areas[i] = faceobjects[i].rect.area();
            areas[i] = contourArea(object_apex_tmp);
        }

        for (int i = 0; i < n; i++)
        {
            BuffArmor &a = faceobjects[i];
            std::vector<cv::Point2f> apex_a(a.apex, a.apex + 5);

            int keep = 1;
            for (int j = 0; j < (int)picked.size(); j++)
            {
                BuffArmor &b = faceobjects[picked[j]];
                std::vector<cv::Point2f> apex_b(b.apex, b.apex + 5);
                std::vector<cv::Point2f> apex_inter;

                // intersection over union
                // float inter_area = intersection_area(a, b);
                // float union_area = areas[i] + areas[picked[j]] - inter_area;

                float inter_area = intersectConvexConvex(apex_a, apex_b, apex_inter);
                float union_area = areas[i] + areas[picked[j]] - inter_area;
                float iou = inter_area / union_area;

                if (iou > nms_threshold || isnan(iou))
                {
                    keep = 0;
                    // Stored for Merge
                    if (iou > MERGE_MIN_IOU && abs(a.prob - b.prob) < MERGE_CONF_ERROR && a.cls == b.cls && a.color == b.color)
                    {
                        // std::cout << "a.color: " << a.color << std::endl;
                        for (int i = 0; i < 5; i++)
                        {
                            // std::cout << "find" << std::endl;
                            b.pts.push_back(a.apex[i]);
                        }
                    }
                    // cout<<b.pts_x.size()<<endl;
                }
            }

            if (keep)
                picked.push_back(i);
        }
    }

    /**
     * @brief Decode outputs.
     * @param prob Original predition output.
     * @param objects Vector of objects predicted.
     * @param img_w Width of Image.
     * @param img_h Height of Image.
     */
    static void decodeOutputs(const float *prob, std::vector<BuffArmor> &objects,
                              Eigen::Matrix<float, 3, 3> &transform_matrix)
    {
        std::vector<BuffArmor> proposals;
        std::vector<int> strides = {8, 16, 32};
        std::vector<GridAndStride> grid_strides;
        generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides);
        generateYoloxProposals(grid_strides, prob, transform_matrix, BBOX_CONF_THRESH, proposals);
        qsort_descent_inplace(proposals);

        if (proposals.size() >= TOPK)
            proposals.resize(TOPK);
        std::vector<int> picked;
        nms_sorted_bboxes(proposals, picked, NMS_THRESH);
        int count = picked.size();
        objects.resize(count);
        for (int i = 0; i < count; i++)
        {
            objects[i] = proposals[picked[i]];
        }
    }

    float calcTriangleArea(cv::Point2f pts[3])
    {
        /**
         * @brief 海伦公式计算三角形面积
         *
         * @param pts 三角形顶点
         * @return float 面积
         */
        auto a = sqrt(pow((pts[0] - pts[1]).x, 2) + pow((pts[0] - pts[1]).y, 2));
        auto b = sqrt(pow((pts[1] - pts[2]).x, 2) + pow((pts[1] - pts[2]).y, 2));
        auto c = sqrt(pow((pts[2] - pts[0]).x, 2) + pow((pts[2] - pts[0]).y, 2));

        auto p = (a + b + c) / 2.f;

        return sqrt(p * (p - a) * (p - b) * (p - c));
    }

    float calcTetragonArea(cv::Point2f pts[4])
    {
        /**
         * @brief 计算四边形面积
         *
         * @param pts 四边形顶点
         * @return float 面积
         */
        return calcTriangleArea(&pts[0]) + calcTriangleArea(&pts[1]);
    }

    BuffFivePoints::BuffFivePoints(const std::string &model_file)
    {
        initModel(model_file);
    }

    BuffFivePoints::~BuffFivePoints()
    {

    }
    
    void BuffFivePoints::initModel(const std::string &model_file)
    {
        std::cout << "Start initialize model..." << std::endl;

        // Setting Configuration Values.
        core.set_property("CPU", ov::enable_profiling(true));

        // Step 1.Create openvino runtime core
        model = core.read_model(model_file);

        // Preprocessing.
        ov::preprocess::PrePostProcessor ppp(model);
        ppp.input().tensor().set_element_type(ov::element::f32);

        // set output precision.
        ppp.output().tensor().set_element_type(ov::element::f32);

        // 将预处理融入原始模型.
        ppp.build();

        // Step 2. Compile the model
        compiled_model = core.compile_model(
            model,
            "CPU",
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
            // "AUTO:GPU,CPU",
            // ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
            // ov::hint::inference_precision(ov::element::u8)
        );

        // compiled_model.set_property(ov::device::priorities("GPU"));

        // Step 3. Create an Inference Request
        infer_request = compiled_model.create_infer_request();
    }

    BLine BuffFivePoints::operator()(const cv::Mat &img)
    {
        if (img.empty())
        {
            std::cout << "[DETECT] ERROR: 传入了空的src";
            return BLine();
        }

        cv::Mat pr_img = scaledResize(img, transfrom_matrix);

        cv::Mat pre;
        cv::Mat pre_split[3];
        pr_img.convertTo(pre, CV_32F);
        cv::split(pre, pre_split);

        // Get input tensor by index
        input_tensor = infer_request.get_input_tensor(0);

        // 准备输入
        infer_request.set_input_tensor(input_tensor);

        float *tensor_data = input_tensor.data<float_t>();

        auto img_offset = INPUT_H * INPUT_W;
        // Copy img into tensor
        for (int c = 0; c < 3; c++)
        {
            memcpy(tensor_data, pre_split[c].data, INPUT_H * INPUT_W * sizeof(float));
            tensor_data += img_offset;
        }

        // 推理
        infer_request.infer();

        // 处理推理结果
        ov::Tensor output_tensor = infer_request.get_output_tensor();
        float *output = output_tensor.data<float_t>();

        std::vector<BuffArmor> objects;
        decodeOutputs(output, objects, transfrom_matrix);

        for (auto object = objects.begin(); object != objects.end(); ++object)
        {
            // 对候选框预测角点进行平均,降低误差
            if ((*object).pts.size() >= 10)
            {
                int N = (*object).pts.size();
                cv::Point2f pts_final[5];
                for (int i = 0; i < N; i++)
                {
                    pts_final[i % 5] += (*object).pts[i];
                }

                for (int i = 0; i < 5; i++)
                {
                    pts_final[i].x = pts_final[i].x / (N / 5);
                    pts_final[i].y = pts_final[i].y / (N / 5);
                }

                (*object).apex[0] = pts_final[0];
                (*object).apex[1] = pts_final[1];
                (*object).apex[2] = pts_final[2];
                (*object).apex[3] = pts_final[3];
                (*object).apex[4] = pts_final[4];
                (*object).corners[0] = pts_final[0];
                (*object).corners[1] = pts_final[1];
                (*object).corners[2] = pts_final[2];
                (*object).corners[3] = pts_final[3];
                (*object).corners[4] = pts_final[4];
            }
        }

        float prob = 0;
        if (objects.size() <= 0)
        {
            aim_target.is_useful = false;
            return BLine();
        }
        else
        {
            for (int i = 0; i < objects.size(); i++)
            {
                if (objects[i].cls == 0)
                {
                    if (objects[i].prob > prob)
                    {
                        prob = objects[i].prob;
                        aim_target = objects[i];
                        aim_target.is_useful = true;
                    }
                }
            }
        }

        BLine bline;
        bline.pts[0] = (aim_target.corners.at(1) + aim_target.corners.at(2) + aim_target.corners.at(3) + aim_target.corners.at(4)) / 4.0;
        bline.pts[1] = aim_target.corners.at(0);

        cv::Point2f armor_line = -(aim_target.corners[2] + aim_target.corners[1]) / 2 + (aim_target.corners[3] + aim_target.corners[4]) / 2;
        bline.angle = atan2(armor_line.x, armor_line.y);
        bline.is_useful = aim_target.is_useful;
        
        return bline;
    }
} //namespace DETECTOR