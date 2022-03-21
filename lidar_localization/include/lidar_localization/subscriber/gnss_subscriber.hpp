#ifndef GNSS_SUBSCRIBER_HPP
#define GNSS_SUBSCRIBER_HPP

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "../sensor_data/gnss_data.hpp"

namespace lidar_localization{
    class GNSSSubscriber
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<GNSSData> new_gnss_data_;

        void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

    public:
        GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        GNSSSubscriber()=default;
        void ParseData(std::deque<GNSSData>& deque_gnss_data);
     
    };        
}
#endif