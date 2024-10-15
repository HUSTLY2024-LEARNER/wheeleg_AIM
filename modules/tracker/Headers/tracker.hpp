#pragma once

#include "TrackingGate2D.hpp"
#include "driver.hpp"

namespace TRACKER
{
    struct AnalyzeResult
    {
        bool yolo_better = true;
    };

    struct TrackingPackage
    {
        std::vector<std::pair<int,BBox>> bboxes_with_index;
        MoveStatusSuspect move_status;
        DRIVER::SerialReadData::IMU_Flag imu_flag;
        long time_stamp;
    };
} //namespace TRACKER