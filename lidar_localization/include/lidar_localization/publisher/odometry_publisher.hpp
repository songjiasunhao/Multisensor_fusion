/*
里程计信息发布
*/
#ifndef ODOMETRY_PUBLISHER_HPP
#define ODOMETRY_PUBLISHER_HPP

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace lidar_localization
{
    class OdometryPublisher
    {
        private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        nav_msgs::Odometry odometry_;//有一部分不变的信息在构造函数

        public:
            OdometryPublisher(ros::NodeHandle& nh, std::string topic_name, std::string base_frame_id, std::string child_frame_id, int buff_size);
            OdometryPublisher() = default;

            void Publisher(const Eigen::Matrix4f& transform_matrix);//防止修改，减少值传参，提高效率，const&也可以接收临时对象
    };    
}
#endif