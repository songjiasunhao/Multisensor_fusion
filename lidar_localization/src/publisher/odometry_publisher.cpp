/*通过ros发布里程计*/

#include "../../include/lidar_localization/publisher/odometry_publisher.hpp"

namespace lidar_localization
{
   OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh, std::string topic_name, std::string base_frame_id, std::string child_frame_id, int buff_size)
   :nh_(nh)//为什么要用size_t?size_t可以转化为int
    {
        publisher_=nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
        odometry_.header.frame_id = base_frame_id;//位置信息由base_frame_id给出
        odometry_.child_frame_id=child_frame_id;//转向信息由child_frame_id给出

    }
    
    void OdometryPublisher::Publisher(const Eigen::Matrix4f& transform_matrix)
    {
        odometry_.header.stamp = ros::Time::now();

        odometry_.pose.pose.position.x = transform_matrix(0,3);
        odometry_.pose.pose.position.y = transform_matrix(1,3);
        odometry_.pose.pose.position.z = transform_matrix(2,3);

        Eigen::Quaternionf q;//float型四元数
        q = transform_matrix.block<3,3>(0,0);//取块操作
        odometry_.pose.pose.orientation.x = q.x();
        odometry_.pose.pose.orientation.y = q.y();
        odometry_.pose.pose.orientation.z = q.z();
        odometry_.pose.pose.orientation.w = q.w();

        publisher_.publish(odometry_);
    }
} // namespace lidar_localization



