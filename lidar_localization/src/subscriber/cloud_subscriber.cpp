/*订阅激光点云信息，并解析数据*/

#include "../../include/lidar_localization/subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization
{
    CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh){
        subscriber_=nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);//第四个函数的作用，为了使用类内函数作回调函数
    }

    void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
    {
        CloudData cloud_data;
        cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
        pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));//格式转换

        new_cloud_data_.push_back(cloud_data);
    }

    void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff)
    {
        if (new_cloud_data_.size()>0)
        {
            cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());//插入新点云
            new_cloud_data_.clear();//清空操作
        }
        
    }
} // namespace lidar_localization
