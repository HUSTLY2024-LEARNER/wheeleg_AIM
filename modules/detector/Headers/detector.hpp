#pragma once

#include "driver.hpp"
#include "BBoxes.h"
#include "BLines.h"

namespace DETECTOR
{
    enum class DetectStatus 
    {
        //armor detect
        ARMOR_NO_TARGET,
        ARMOR_ONE_TARGET,
        ARMOR_MULTI_TARGET,

        //buff detect
        BUFF_NOT_FOUND,
        BUFF_ONLY_UNACTIVE,
        BUFF_ONLY_ACTIVE,
        BUFF_FOUND_BOTH
    };

    struct DetectionPackage
    {
        DetectStatus detect_status;
        DRIVER::SerialReadData::IMU_Flag imu_flag;
        BBoxes armor_detection;
        BLines buff_detection;
        long time_stamp;
    };

    struct ArmorDetectionPackage
    {
        DRIVER::SerialReadData::IMU_Flag imu_flag;
        BBoxes armor_detection;
        long time_stamp;
    };

    struct BuffDetectionPackage
    {
        DRIVER::SerialReadData::IMU_Flag imu_flag;
        BLine buff_detection;
        long time_stamp;
    };

    struct TwoDetections
    {
        BBoxes yolo_detection;
        BBoxes tradition_detection;
    };

    struct SingleDetection
    {
        BBoxes single_detection;
        uint8_t aim_request;
    };
    
} //namespace DETECTOR