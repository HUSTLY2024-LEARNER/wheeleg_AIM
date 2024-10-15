#include "MultiPredictor.hpp"

namespace PREDICTOR
{
    void NotPredictor::rebootPredictor()
    {

    }
    
    void NotPredictor::runPredictor(SOLVER::IndexedArmorPoses& armor_poses,TRACKER::MoveStatusSuspect& move_status,double& update_time)
    {
        my_points.clear();
        for(auto pose : armor_poses)
        { 
            Eigen::Vector3d temp_(pose.second.translate[0], pose.second.translate[2], -pose.second.translate[1]);
            my_points.push_back(temp_);
        }
    }

    std::vector<Eigen::Vector3d> NotPredictor::getAimPointsAfterBulletFly(const float& delay_t, FollowMode follow_mode)
    {
        return my_points;
    }

    FollowMode NotPredictor::getFollowMode()
    {
        
    }

    SpinState NotPredictor::getSpinState()
    {
        
    }

    CenterAndRange NotPredictor::getAimBounds()
    {

    }
}