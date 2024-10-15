#pragma once

#include <vector>

#include <BBoxes.h>
#include <BLines.h>

using namespace DETECTOR;
namespace TRACKER
{
    // 识别比较器基类
    class MethodComparator
    {
    protected :
        MethodComparator(): yolo_better_than_tradition(true){};

    public:
        virtual ~MethodComparator() = default;


        // 父类接口调用纯虚函数
        void analyzeSingle(const std::vector<BBox>& bboxes, const std::pair<uint8_t,uint8_t>& flag_pair) const {
            analyzeSingleImpl(bboxes,flag_pair);
        }

        void analyzeSingle(const std::vector<BLine>& blines, const std::pair<uint8_t,uint8_t>& flag_pair) const {
            analyzeSingleImpl(blines,flag_pair);
        }

        void analyzeDouble(const std::vector<BBox>& bboxes_1, const std::vector<BBox>& bboxes_2) const {
            analyzeDoubleImpl(bboxes_1,bboxes_2);
        }

        void analyzeDouble(const std::vector<BLine>& blines_1, const std::vector<BLine>& blines_2) const {
            analyzeDoubleImpl(blines_1,blines_2);
        }

        // true : yolo better | false : tradition better
        bool getComparison() const
        {
            return this->yolo_better_than_tradition;
        }
    private:
        // 纯虚函数，待在子类中实现
        virtual void analyzeSingleImpl(const std::vector<BBox>& bboxes, const std::pair<uint8_t,uint8_t>& flag_pair) const = 0;
        virtual void analyzeSingleImpl(const std::vector<BLine>& blines, const std::pair<uint8_t,uint8_t>& flag_pair) const = 0;

        virtual void analyzeDoubleImpl(const std::vector<BBox>& bboxes_1, const std::vector<BBox>& bboxes_2) const = 0;
        virtual void analyzeDoubleImpl(const std::vector<BLine>& blines_1, const std::vector<BLine>& blines_2) const = 0;

        bool yolo_better_than_tradition;
    };

    // Armor 比较器
    class ArmorComparator : public MethodComparator
    {
    private:
        void analyzeSingleImpl(const std::vector<BBox>& bboxes, const std::pair<uint8_t,uint8_t>& flag_pair) const override;
    };

    // Buff 比较器
    class BuffComparator : public MethodComparator
    {
    private:
        void analyzeSingleImpl(const std::vector<BLine>& blines, const std::pair<uint8_t,uint8_t>& flag_pair) const override;
    };

} // namespace TRACKER
