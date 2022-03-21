//
// Created by sunhao on 2021/7/27.经纬度转化
//
#include "../../include/lidar_localization/sensor_data/gnss_data.hpp"
#include "glog/logging.h"//日志信息输出

//静态变量必须在类外初始化，且必须在类使用前初始化
bool lidar_localization::GNSSData::origin_position_inited =false;
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_convert;

namespace lidar_localization{
        void GNSSData::InitOriginPosition(){
            geo_convert.Reset(latitude,longitude,altitude);//经纬度原点初始化,将此处作为原点，ENU坐标系原点重置
            origin_position_inited =true;
        }

        void GNSSData::UpdateXYZ(){
            if (!origin_position_inited)
            {
                LOG(WARNING) <<"经纬度原点未初始化";
            }
            geo_convert.Forward(latitude,longitude,altitude,local_E,local_N,local_U);//将经纬度转化为ENU坐标
        }
}

