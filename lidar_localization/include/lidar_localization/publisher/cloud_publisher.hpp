/*
在ros中发布点云信息
*/
#ifndef CLOUD_PUBLISHER_HPP
#define CLOUD_PUBLISHER_HPP

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "../sensor_data/cloud_data.hpp"

namespace lidar_localization{
    class CloudPublisher
    {
    public:
        CloudPublisher(ros::NodeHandle& nh, std::string topic_name, size_t buff_size ,std::string frame_id);//需要用到一些其他参数，所以需要新构造函数？
        CloudPublisher() = default;//默认构造函数有必要吗
        void Publish(CloudData::CLOUD_PTR cloud_ptr_input);
    
    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;//用于表示和此数据关联的帧，在坐标系变化中可以理解为数据所在的坐标系名称
    };
}
#endif