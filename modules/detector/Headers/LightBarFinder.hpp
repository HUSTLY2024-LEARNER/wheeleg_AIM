#pragma once

#include <opencv2/opencv.hpp>

namespace DETECTOR
{
    typedef std::vector<cv::RotatedRect> LightBarBlobs;
    
    class LightBarFinder
    {
    public:
        explicit LightBarFinder(int&);
        ~LightBarFinder();
        std::vector<cv::Point2f> operator()(const cv::Mat &roi);
    private:
        void setEnemyColor(int&);
        int enemy_color;
        static inline bool isValidLightBarBlob(const cv::RotatedRect &);
        static inline bool isRightColor();
    };
}