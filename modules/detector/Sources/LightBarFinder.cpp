#include "LightBarFinder.hpp"

namespace DETECTOR
{
    LightBarFinder::LightBarFinder(int& enemy_color)
    {
        setEnemyColor(enemy_color);  
    }

    LightBarFinder::~LightBarFinder()
    {

    }

    std::vector<cv::Point2f> LightBarFinder::operator()(const cv::Mat &roi)
    {
        cv::Mat gray_img;
        cv::cvtColor(roi, gray_img, cv::COLOR_BGR2GRAY);
    }

    void LightBarFinder::setEnemyColor(int&)
    {

    }
} // namespace DETECTOR
