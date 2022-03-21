/*订阅GNSS数据，并解析处理*/

#include "../../include/lidar_localization/subscriber/gnss_subscriber.hpp"




namespace lidar_localization{

        GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
        :nh_(nh)
        {
            subscriber_=nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
        }
    
       void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr)
       {
           GNSSData gnss_data;
           gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();//时间戳转换成浮点型格式
           gnss_data.latitude = nav_sat_fix_ptr->latitude;//经度
           gnss_data.longitude = nav_sat_fix_ptr->longitude;//纬度
           gnss_data.altitude = nav_sat_fix_ptr->altitude;//海拔
           gnss_data.status = nav_sat_fix_ptr->status.status;//卫星定位状态，是否定位成功
           gnss_data.service = nav_sat_fix_ptr->status.service;//定义接收机使用了哪种全球导航卫星系统信号

           new_gnss_data_.push_back(gnss_data);
       }

       void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff){
           if (new_gnss_data_.size()>0)
           {
               gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
               new_gnss_data_.clear();
           }
       }
}