#ifndef CLOUD_SUBSCRIBER_HPP
#define CLOUD_SUBSCRIBER_HPP
 
#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "../sensor_data/cloud_data.hpp"

namespace lidar_localization
{
    class CloudSubscriber
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<CloudData> new_cloud_data_;//包括一个类，里面有time和CLOUD_PTR
        
        void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

    public:
        CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        CloudSubscriber() = default;//默认构造函数，会默认初始化成员变量
        void ParseData(std::deque<CloudData>& deque_cloud_data);
     
    };
} // namespace lidar_localization

 
#endif
