#ifndef TF_LISTENER_HPP
#define TF_LISTENER_HPP

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace lidar_localization{
    class TFListener
    {
    private:
        ros::NodeHandle nh_;
        tf::TransformListener listener_;
        std::string base_frame_id_;
        std::string child_frame_id_;

        bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);

    public:
        TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
        TFListener() = default;

        bool LookupData(Eigen::Matrix4f& transform_matrix);


    }; 
}
#endif