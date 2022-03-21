#ifndef IMU_SUBSCRIBER_HPP
#define IMU_SUBSCRIBER_HPP

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "../sensor_data/imu_data.hpp"

namespace lidar_localization{
    class IMUSubscriber
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<IMUData> new_imu_data_;

        void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    public:
        IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        IMUSubscriber()= default;

        void ParseDate(std::deque<IMUData>& deque_imu_data);
    };
   
    
}
#endif