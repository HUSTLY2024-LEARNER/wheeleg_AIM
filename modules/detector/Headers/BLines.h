#pragma once

#include "opencv2/core.hpp"
#include <vector>

namespace DETECTOR
{
    class BLine
    {
    public:
        cv::Point2f pts[2]; // fan, r
        float confidence;
        int color_id;
        int tag_id;
        float angle;
        bool is_useful = false;
    };

    typedef std::vector<BLine> BLines;
} //namespace DETECTOR
