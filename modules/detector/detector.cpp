#include <thread>
#include <chrono>
#include <type_traits>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <umt.hpp>
#include <detector.hpp>
#include <ArmorOneStage.hpp>
#include <ArmorTwoStage.hpp>
#include <BuffFivePoints.hpp>
#include <BuffBBox.hpp>
#include <utils.h>

using namespace LY_UTILS;

#define JUDGE_POINT 2.6

namespace DETECTOR
{

#define AIMING_ARMOR(x) (x->mode_want==static_cast<uint8_t>(DRIVER::AimMode::AIM_ARMOR))
#define AIMING_BUFF(x) (x->mode_want==static_cast<uint8_t>(DRIVER::AimMode::AIM_LARGE_BUFF)||x->mode_want==static_cast<uint8_t>(DRIVER::AimMode::AIM_SMALL_BUFF))

    template <typename DetectionType>
    DetectStatus judgeDetectStatus(const std::vector<DetectionType>& detectionResults) {
        static_assert(std::is_same_v<DetectionType, BBox> || std::is_same_v<DetectionType, BLine>,
                    "DetectionType must be either BBox or BLine");
        if constexpr (std::is_same_v<DetectionType, BLine>) {
            // 处理 BLine 的逻辑
            if (detectionResults.empty()) {
                return DetectStatus::BUFF_NOT_FOUND;
            } else if (detectionResults.size() == 1) {
                return DetectStatus::BUFF_ONLY_UNACTIVE;
            } else {
                return DetectStatus::BUFF_ONLY_ACTIVE;
            }
        } else if constexpr (std::is_same_v<DetectionType, BBox>) {
            // 处理 BBox 的逻辑
            if (detectionResults.empty()) {
                return DetectStatus::ARMOR_NO_TARGET;
            } else if (detectionResults.size() == 1) {
                return DetectStatus::ARMOR_ONE_TARGET;
            } else {
                return DetectStatus::ARMOR_MULTI_TARGET;
            }
        } else {
            //static_assert(std::always_false<DetectionType>, "Unsupported DetectionType");
        }
    }

    ARMOR_SIZE armor_size_judge(cv::Point2f* corners_array)
    {
        std::function<float(cv::Point, cv::Point)> calculate_distance = [](cv::Point p1, cv::Point p2) {
            return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));};

        float l_left = calculate_distance(corners_array[0],corners_array[1]);
        float l_right = calculate_distance(corners_array[2],corners_array[3]);
        float d_up = calculate_distance(corners_array[0],corners_array[3]);
        float d_down = calculate_distance(corners_array[1],corners_array[2]);

        float denominator = 1/ (l_left + l_right);
        float numerator = d_up + d_down;
        float rate = denominator * numerator;

        float alpha = (l_left + l_right)*0.002f;
        float theta = std::min(l_left,l_right)/std::max(l_left,l_right);
        float confidence = 1/(1+std::exp(-22*(theta-(0.85-alpha))));

        int size_class = (rate > JUDGE_POINT) ? 1 : -1;

        if(confidence > 0.8)
        {
            COUT("CONFIDENCE "<<confidence<<" for "<<size_class, BLUE);
        }
        return ARMOR_SIZE::UNKNOW;
    }

    void detection_run(const std::string &armor_model_file_one_stage,const std::string &armor_model_file_two_stage,const std::string &buff_model_file)
    {
        //ARMOR
        //one stage inferer
        std::unique_ptr<ArmorOneStage> armor_one_stage_inferer = std::make_unique<ArmorOneStage>(armor_model_file_one_stage);
        //two stage inferer
        //std::unique_ptr<ArmorTwoStage> armor_two_stage_inferer = std::make_unique<ArmorTwoStage>(armor_model_file_two_stage, enemy_color);

        //BUFF
        //bbox inferer
        // std::unique_ptr<BuffBBox> buff_bbox_inferer = std::make_unique<BuffBBox>(buff_model_file);
        //five points inferer
        std::unique_ptr<BuffFivePoints> buff_five_points_inferer = std::make_unique<BuffFivePoints>(buff_model_file);

        umt::Subscriber<DRIVER::PicStamp> pic_sub("image_raw");
        umt::Publisher<DRIVER::PicStamp> pic_pub("image_used");

        auto imu_flag_data = umt::ObjManager<DRIVER::SerialReadData::IMU_Flag>::find_or_create("imu_flag_data");

        // 为armor_size_map添加键值对
        auto armor_size_map = umt::ObjManager<ARMOR_SIZE_MAP>::find_or_create("armor_size_map");
        armor_size_map.get()->insert({
            {ENEMY_TYPE::Hero, ARMOR_SIZE::BIG_ARMOR},
            {ENEMY_TYPE::Engineer, ARMOR_SIZE::SMALL_ARMOR},
            {ENEMY_TYPE::Infantry3, ARMOR_SIZE::SMALL_ARMOR},
            {ENEMY_TYPE::Infantry4, ARMOR_SIZE::SMALL_ARMOR},
            {ENEMY_TYPE::Infantry5, ARMOR_SIZE::BIG_ARMOR},
            {ENEMY_TYPE::Sentry, ARMOR_SIZE::SMALL_ARMOR},
            {ENEMY_TYPE::Outpost, ARMOR_SIZE::SMALL_ARMOR},
            {ENEMY_TYPE::Base, ARMOR_SIZE::BIG_ARMOR}
        });

        std::set<int> infantry_check_set = {static_cast<int>(ENEMY_TYPE::Infantry3), static_cast<int>(ENEMY_TYPE::Infantry4), static_cast<int>(ENEMY_TYPE::Infantry5)};

        auto serial_work_ok = umt::ObjManager<DRIVER::SerialOK>::find_or_create("serial_work_ok");

        umt::Publisher<DetectionPackage> detection_pub("detection_pack");
        umt::Publisher<ArmorDetectionPackage> armor_detection_pub("armor_detection_pack");
        umt::Publisher<BuffDetectionPackage> buff_detection_pub("buff_detection_pack");

        umt::Publisher<TwoDetections> two_analyze_pub("two_detections");
        umt::Publisher<SingleDetection> single_analyze_pub("single_detection");

        //auto analyze_result = umt::ObjManager<TRACKER::AnalyzeResult>::find_or_create("analyze_reslut");
        typedef struct {bool yolo_better;} __temp__Struct__u_no_want;
        
        auto analyze_result = std::shared_ptr<__temp__Struct__u_no_want>{};


        while(true)
        {
            DetectStatus this_frame_detection_status;

            auto *this_serial_work_ok = serial_work_ok.get();
            if(!this_serial_work_ok->serial_ok){ COUT("serial work not ok",RED); std::this_thread::sleep_for(std::chrono::milliseconds(1000)); continue;}

            // (*armor_one_stage_inferer).setColorFlag(imu_flag_data.get()->enemy_color);
            (*armor_one_stage_inferer).setColorFlag(0);


            try {
                auto pic_stamp = pic_sub.pop();
                
                auto *this_frame_imu_flag_data = imu_flag_data.get();

                std::cout<<"yaw:"<<this_frame_imu_flag_data->yaw_now<<std::endl;
                std::cout<<"pitch:"<<this_frame_imu_flag_data->pitch_now<<std::endl;
                if(AIMING_ARMOR(this_frame_imu_flag_data)){
                    // BBoxes yolo_detection, tradition_detection;
                    // yolo_detection = (*armor_one_stage_inferer)(pic_stamp.pic);
                    // tradition_detection = (*armor_two_stage_inferer)(pic_stamp.pic);
                    // two_analyze_pub.push({yolo_detection,tradition_detection});

                    //TODO:替换成大尺寸模型识别
                    BBoxes detection;
                    detection = (*armor_one_stage_inferer)(pic_stamp.pic);
                    std::string detected_numbers;
                    for(auto detect:detection)
                    {
                        detected_numbers += std::to_string(detect.tag_id)+ ", ";
                        if(infantry_check_set.count(detect.tag_id) > 0)
                        {
                            //armor_size_map.get()->at(static_cast<ENEMY_TYPE>(detect.tag_id)) = armor_size_judge(detect.corners);
                            armor_size_map.get()->at(static_cast<ENEMY_TYPE>(detect.tag_id)) = ARMOR_SIZE::SMALL_ARMOR;
                        }
                    } 
                    COUT(detection.size()<<" armors detected: "<<detected_numbers,BLUE);
                    
                    this_frame_detection_status = judgeDetectStatus(detection);
                    
                    detection_pub.push(DetectionPackage{this_frame_detection_status,*this_frame_imu_flag_data,detection,BLines(),pic_stamp.time_stamp});
                }
                else if(AIMING_BUFF(this_frame_imu_flag_data)){
                    BLine detection;
                    detection = (*buff_five_points_inferer)(pic_stamp.pic);
                    if(detection.is_useful)
                    {
                        COUT("DETECTED UNACTIVE FAN", GREEN);
                        buff_detection_pub.push(BuffDetectionPackage{*this_frame_imu_flag_data,detection,pic_stamp.time_stamp});
                    }
                    else
                    {
                        COUT("NO UNACTIVE FAN", RED);
                    }
                }
                //autoaim suspend
                if(this_frame_imu_flag_data->aim_request == 0){
                    //操作手没有发送辅瞄请求,可以进行耗时较长的操作
                    //1.大尺寸模型识别(TODO)
                    //2.深度学习与传统视觉对比/融合，即双端识别的卡尔曼滤波
                    COUT("NO AIM REQUEST",YELLOW);
                } else{  //autoaim taking over
                    COUT("AUTOAIM TAKE OVER",YELLOW);
                }

                pic_pub.push(pic_stamp);

            } catch (umt::MessageError &e) {
                COUT("[WARNING] 'image_raw' {}\n"<<e.what(),RED);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
    }

    void bkg_detector_run(const std::string &armor_model_file_one_stage,const std::string &armor_model_file_two_stage,const std::string &buff_model_file){
        std::thread([=](){
            detection_run(armor_model_file_one_stage,armor_model_file_two_stage,buff_model_file);
        }).detach();
    }

namespace py = pybind11;
PYBIND11_EMBEDDED_MODULE(DETECTOR_, m) {
    m.def("bkg_detector_run", bkg_detector_run, py::arg("armor_model_file"), py::arg("armor_model_file"), py::arg("buff_model_file"));
}

} //namespace DETECTOR