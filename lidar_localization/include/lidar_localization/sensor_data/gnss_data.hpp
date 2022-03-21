//
// Created by sunhao on 2021/7/27.
//

#ifndef SRC_GNSS_DATA_H
#define SRC_GNSS_DATA_H
#include <vector>
#include <string>

#include "Geocentric/LocalCartesian.hpp"//功能就是把椭球体下的地理坐标经纬海系坐标转为ENU局部系下的坐标

using std::vector;
using std::string;//后边不用加std了

namespace lidar_localization{
    class GNSSData
    {
    private:
        static GeographicLib::LocalCartesian  geo_convert;//定义一个转化坐标的对象
        static bool origin_position_inited;//静态变量属于整个类
    public:
        double time =0.0;
        double longitude =0.0;//经度
        double latitude =0.0;//纬度
        double altitude =0.0;//海拔；
        double local_E =0.0;//东
        double local_N =0.0;//北
        double local_U =0.0;//天
        int status =0;//卫星定位状态，是否增强
        int service =0;//定义接收机使用了哪种全球导航卫星系统信号

    
        void InitOriginPosition();
        void UpdateXYZ();
    };
}


#endif //SRC_GNSS_DATA_H
//