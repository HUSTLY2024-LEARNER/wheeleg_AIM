#include "MethodComparator.hpp"

namespace TRACKER
{

    /*当前模型分析器*/

    //对于装甲板识别来说
    //如果aim_request==1
    //即操作手正在按右键，期望辅瞄接管
    //如果is_shoot==1但aim_request==0
    //即操作手正在打弹但是并没有开启辅瞄
    //若此时未识别到目标的时间超过一个阈值，认为模型出错
    //若掉帧严重、噪声明显，也认为模型出错    
    void ArmorComparator::analyzeSingleImpl(const BBoxes& bboxes, const std::pair<uint8_t,uint8_t>& flag_pair) const
    {
        //hope autoaim to take control
        if(static_cast<int>(flag_pair.first)==1&&static_cast<int>(flag_pair.second)==1){

        }
    }

    //对于能量机关识别来说
    //如果aim_request==1
    //即操作手已经进入了打符模式并且按下右键，期望辅瞄拟合
    //若此时未识别到目标的时间超过一个阈值，认为模型出错
    //若掉帧严重、噪声明显，也认为模型出错    
    void BuffComparator::analyzeSingleImpl(const BLines& blines, const std::pair<uint8_t,uint8_t>& flag_pair) const
    {

    }

    /**************/

    /*双模型比较器*/

    

} // namespace TRACKER