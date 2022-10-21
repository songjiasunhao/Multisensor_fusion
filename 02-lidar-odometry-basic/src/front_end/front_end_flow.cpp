/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/front_end/front_end_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/dataset/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitSubscribers(nh, config_node["measurements"]);

    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", 100, "/map");
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", 100, "/map");
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);

    front_end_ptr_ = std::make_shared<FrontEnd>();

    local_map_ptr_.reset(new CloudData::CLOUD());
    global_map_ptr_.reset(new CloudData::CLOUD());
    current_scan_ptr_.reset(new CloudData::CLOUD());
}

bool FrontEndFlow::Run() {
    if (!ReadData()) {
        return false;
    }
        
    if (!InitCalibration())  {
        return false;
    }

    while(HasData()) {
        if (!ValidData()) {
            continue;
        }
 
        //std::cout <<current_cloud_data_.cloud_ptr->size()<<std::endl;
        
       // std::cout<< current_imu_data_.linear_acceleration.x<<std::endl;
        if (UpdateLaserOdometry()) {

            PublishData();
            SaveTrajectory();
        } else {
            LOG(INFO) << "UpdateLaserOdometry failed!" << std::endl;
        }
    }

    return true;
}

bool FrontEndFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    //
    // init TF sub & pub:数据集里没有tf所以需要
    //
    lidar_to_imu_tf_sub_ptr_ = std::make_shared<TFListener>(
        nh, 
        config_node["imu"]["frame_id"].as<std::string>(), config_node["lidar"]["frame_id"].as<std::string>()
    );
    lidar_to_map_tf_pub_ptr_ = std::make_shared<TFBroadCaster>(
        "/map", config_node["lidar"]["frame_id"].as<std::string>()
    );
   std::cout<<"finish init TFpub";
    //
    // init input message subs:
    // 
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(
        nh, 
        config_node["lidar"]["topic_name"].as<std::string>(), config_node["lidar"]["queue_size"].as<int>()
    );
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(
        nh, 
        config_node["imu"]["topic_name"].as<std::string>(), config_node["imu"]["queue_size"].as<int>()
    );

    return true;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    //static std::deque<IMUData> unsynced_imu_;
    
    imu_sub_ptr_->ParseData(imu_data_buff_);
    std::cout<<"cloud "<<cloud_data_buff_.size()<<std::endl;
 std::cout<<"imu "<<imu_data_buff_.size()<<std::endl;
    if (cloud_data_buff_.size() == 0) {
        std::cout<<"cloud==0"<<std::endl;
        return false;
    }
        
    //double cloud_time = cloud_data_buff_.front().time; IMU与点云的时间同步有问题 会导致点云数据计算慢 崩溃等问题 先注释
    

   /* bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
     std::cout<<"imu buff"<<imu_data_buff_.size()<<std::endl;
    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu ) {
            cloud_data_buff_.pop_front();
            printf("imu data invalid");
            return false;
        }
        sensor_inited = true;
    }
*/
    
    return true;
}

bool FrontEndFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_tf_sub_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool FrontEndFlow::InitGNSS() {
    static bool gnss_inited = false;
    
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool FrontEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
        
    return true;
}

bool FrontEndFlow::ValidData() {
 /* current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();

    double d_time = current_cloud_data_.time - current_imu_data_.time;
    if (d_time < -0.05) {//
        cloud_data_buff_.pop_front();
        return false;
    }

    if (d_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }
 */
    current_cloud_data_ = cloud_data_buff_.back();
    current_imu_data_ = imu_data_buff_.back();

    cloud_data_buff_.clear();
 
    //cloud_data_buff_.pop_front();
    imu_data_buff_.clear();

    return true;
}

bool FrontEndFlow::UpdateGNSSOdometry() {
    gnss_odometry_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0,3) = current_gnss_data_.local_E;
    gnss_odometry_(1,3) = current_gnss_data_.local_N;
    gnss_odometry_(2,3) = current_gnss_data_.local_U;
    gnss_odometry_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    
    gnss_odometry_ *= lidar_to_imu_;

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool front_end_pose_inited = false;

    if (!front_end_pose_inited) {
        front_end_pose_inited = true;
           Eigen::Matrix4f odometry_ = Eigen::Matrix4f::Identity();
//如果初始值根据实际 
        front_end_ptr_->SetInitPose(odometry_);//laser_odometry_
    }

    laser_odometry_ = Eigen::Matrix4f::Identity();
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool FrontEndFlow::PublishData() {

    lidar_to_map_tf_pub_ptr_->SendTransform(laser_odometry_, current_cloud_data_.time);
 
    laser_odom_pub_ptr_->Publish(laser_odometry_);

    front_end_ptr_->GetCurrentScan(current_scan_ptr_);
    cloud_pub_ptr_->Publish(current_scan_ptr_);

    if (front_end_ptr_->GetNewLocalMap(local_map_ptr_))
        local_map_pub_ptr_->Publish(local_map_ptr_);

    return true;
}

bool FrontEndFlow::SaveTrajectory() {
    static std::ofstream ground_truth, laser_odom;
    static bool is_file_created = false;

    if (!is_file_created) {
        if (!FileManager::CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory"))
            return false;
        if (!FileManager::CreateFile(ground_truth, WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt"))
            return false;
        if (!FileManager::CreateFile(laser_odom, WORK_SPACE_PATH + "/slam_data/trajectory/laser_odom.txt"))
            return false;
        is_file_created = true;
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
     //       ground_truth << gnss_odometry_(i, j);
            laser_odom << laser_odometry_(i, j);
            if (i == 2 && j == 3) {
      //          ground_truth << std::endl;
                laser_odom << std::endl;
            } else {
      //          ground_truth << " ";
                laser_odom << " ";
            }
        }
    }

    return true;
}

bool FrontEndFlow::SaveMap() {
    return front_end_ptr_->SaveMap();
}

bool FrontEndFlow::PublishGlobalMap() {
    if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_)) { 
        global_map_pub_ptr_->Publish(global_map_ptr_);
        global_map_ptr_.reset(new CloudData::CLOUD());
    }

    return true;
}

}