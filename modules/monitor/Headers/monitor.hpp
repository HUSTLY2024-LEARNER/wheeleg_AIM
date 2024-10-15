#pragma once

#include <string>
#include <opencv2/opencv.hpp>


namespace MONITOR {
    class Recorder{
    private:
        cv::VideoWriter writer;
        int mode = writer.fourcc('M', 'P', '4', '2');
        bool is_writer_start = false;
        std::chrono::steady_clock::time_point writer_start_time;
    public:
        void write(const cv::Mat& frame);
    };
}


