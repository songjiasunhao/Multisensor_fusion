/*测试数据节点*/
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

#include "../include/lidar_localization/global_defination/global_defination.h"
#include "../include/lidar_localization/subscriber/cloud_subscriber.hpp"
#include "../include/lidar_localization/subscriber/imu_subscriber.hpp"
#include "../include/lidar_localization/subscriber/gnss_subscriber.hpp"
#include "../include/lidar_localization/tf_listener/tf_listener.hpp"
#include "../include/lidar_localization/publisher/cloud_publisher.hpp"
#include "../include/lidar_localization/publisher/odometry_publisher.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";//设置日志输出目录
    FLAGS_alsologtostderr  =1;//当这个全局变量为真时，忽略FLAGS_stderrthreshold的限制，所有信息打印到终端

    ros::init( argc, argv, "test_frame_node");
    ros::NodeHandle nh;

//为何要使用智能指针，直接调用的坏处
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
  //  std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "velo_link", "imu_link");//原始代码,总感觉反了，有错误再说，因为是固定的转化所以看不太出来
    std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "imu_link", "velo_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);

    std::deque<CloudData> cloud_data_buff; 
    std::deque<IMUData> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;

    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();//
    bool transform_received = false;
    bool gnss_origin_position_inited = false;

    ros::Rate rate(100);//100HZ
    while (ros::ok())
    {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff);
        imu_sub_ptr->ParseDate(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        if(!transform_received)//
        {
            if(lidar_to_imu_ptr->LookupData(lidar_to_imu))
            {
                transform_received = true;
                 LOG(INFO) << "lidar to imu transform matrix is:" << std::endl << lidar_to_imu;
            }
        }else
        {
            while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0)
            {
                CloudData cloud_data = cloud_data_buff.front();
                IMUData imu_data = imu_data_buff.front();
                GNSSData gnss_data = gnss_data_buff.front();//第一个数据

                //时间同步
                double d_time = cloud_data.time - imu_data.time;
                if (d_time < -0.05) {
                    cloud_data_buff.pop_front();
                } else if (d_time > 0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                } else 
                    {
                        cloud_data_buff.pop_front();
                        imu_data_buff.pop_front();
                        gnss_data_buff.pop_front();

                        Eigen::Matrix4f odometry_matrix;
                         Eigen::Matrix4f odometry_matrix_d;//

                        if (!gnss_origin_position_inited) {
                            gnss_data.InitOriginPosition();
                            gnss_origin_position_inited = true;
                        }
                        gnss_data.UpdateXYZ();
                        odometry_matrix(0,3) = gnss_data.local_E;
                        odometry_matrix(1,3) = gnss_data.local_N;
                        odometry_matrix(2,3) = gnss_data.local_U;
                        odometry_matrix.block<3,3>(0,0) = imu_data.GetOrientationMatrix();
                       odometry_matrix *= lidar_to_imu;//Twi*Til=Twl即雷达到世界坐标系的


                        pcl::transformPointCloud(*cloud_data.cloud_ptr,*cloud_data.cloud_ptr, odometry_matrix); //点云变换到世界坐标系

                        cloud_pub_ptr->Publish(cloud_data.cloud_ptr);
                        odom_pub_ptr->Publisher(odometry_matrix);
                    }
            }
        }
            
        
        rate.sleep();
    }
    return 0;
}