#include "ArmorTwoStage.hpp"

namespace DETECTOR
{
    ArmorTwoStage::ArmorTwoStage(const std::string&model_file, const std::string &color)
    {
        if(color == "red")
        {
            this->color_flag = 1;
        }
        else if(color == "blue")
        {
            this->color_flag = 0;
        }
        else
        {
            std::cerr<<"wrong color flag has been input";
        }
        initModel(model_file);
        lightbar_finder = std::make_unique<LightBarFinder>(this->color_flag);
        number_classifier = std::make_unique<NumberClassifier>("svm");
    }

    ArmorTwoStage::~ArmorTwoStage()
    {

    }

    void ArmorTwoStage::initModel(const std::string &model_file)
    {
        // Load the Inference Engine
        ie = ov::Core();
        // Load the network using OpenVINO
        compiled_model = ie.compile_model(model_file, "CPU");
        infer_request = compiled_model.create_infer_request();
    }

    cv::Mat ArmorTwoStage::letterbox(const cv::Mat& source)
    {
        int col = source.cols;
        int row = source.rows;
        int _max = MAX(col, row);
        cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
        source.copyTo(result(cv::Rect(0, 0, col, row)));
        return result;
    }

    cv::Rect ArmorTwoStage::getROI(cv::Mat img, int x1, int x2 ,int y1, int y2)
    {
        int width = x2 - x1;
        int height = y2 - y1;
        int new_x1 = x1 - width * (p_width - 1);
        int new_y1 = y1 - height * (p_height - 1);
        int new_x2 = x2 + width * (p_width - 1);
        int new_y2 = y2 + height * (p_height - 1);
        if (new_x1 < 0) new_x1 = 0;
        if (new_y1 < 0) new_y1 = 0;
        if (new_x2 > img.cols) new_x2 = img.cols;
        if (new_y2 > img.rows) new_y2 = img.rows;
        return cv::Rect(new_x1, new_y1, new_x2 - new_x1, new_y2 - new_y1);
    }

    BBoxes ArmorTwoStage::operator()(const cv::Mat &img)
    {
        // Preprocess the image
        cv::Mat letterbox_img = letterbox(img);
        float scale = letterbox_img.size[0] / 640.0;
        cv::Mat blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true);
        // -------- Step 5. Feed the blob into the input node of the Model -------
        // Get input port for model with one input
        auto input_port = compiled_model.input();
        // Create tensor from external memory
        ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));
        // Set input tensor for model with one input
        infer_request.set_input_tensor(input_tensor);

        // -------- Step 6. Start inference --------
        infer_request.infer();
        // -------- Step 7. Get the inference result --------
        output_tensor = infer_request.get_output_tensor(0);
        output_shape = output_tensor.get_shape();
        auto rows = output_shape[2];        //8400
        auto dimensions = output_shape[1];  //84: box[cx, cy, w, h]+80 classes scores

        // -------- Step 8. Postprocess the result --------
        auto* data = output_tensor.data<float>();
        cv::Mat output_buffer(dimensions, rows, CV_32F, data);
        transpose(output_buffer, output_buffer); //[8400,84]

        std::vector<int> class_ids;
        std::vector<float> class_scores;
        std::vector<cv::Rect> boxes;

        // Figure out the bbox, class_id and class_score
        int outputBufferRows = output_buffer.rows;
        for (int i = 0; i < outputBufferRows; i++) {
            cv::Mat classes_scores = output_buffer.row(i).colRange(4, dimensions);
            cv::Point class_id;
            double maxClassScore;
            minMaxLoc(classes_scores, 0, &maxClassScore, 0, &class_id);

            if (maxClassScore > score_threshold) {
                class_scores.push_back(maxClassScore);
                class_ids.push_back(class_id.x);
                float cx = output_buffer.at<float>(i, 0);
                float cy = output_buffer.at<float>(i, 1);
                float w = output_buffer.at<float>(i, 2);
                float h = output_buffer.at<float>(i, 3);
                int left = int((cx - 0.5 * w) * scale);
                int top = int((cy - 0.5 * h) * scale);
                int width = int(w * scale);
                int height = int(h * scale);

                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
        //NMS
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, class_scores, score_threshold, nms_threshold, indices);

        std::vector<cv::Rect> rois;
        cv::Mat draw_img = img.clone();
        for (size_t i = 0; i < indices.size(); i++) {
            int index = indices[i];
            int x1 = boxes[index].tl().x;
            int y1 = boxes[index].tl().y; //top left
            int x2 = boxes[index].br().x;
            int y2 = boxes[index].br().y; // bottom right
            int color_id = class_ids[index];
            int score = class_scores[index];
            if(color_id == color_flag)
            {
                rois.emplace_back(getROI(img, x1, x2, y1, y2));
            }
        }

        BBoxes bboxes;
        //LightBar Finder + Number Classifier
        for(auto roi : rois)
        {
            std::vector<cv::Point2f> lightbar_points = (*lightbar_finder)(img(roi));
            if(lightbar_points.size() == 4)
            {
                std::pair<int, double> result = number_classifier->predict(img,lightbar_points);
            }
        }
        return BBoxes();
    }
} //namespace DETECTOR