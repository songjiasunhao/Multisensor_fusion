#ifndef CLOUD_DATA_HPP
#define CLOUD_DATA_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_localization{
    class CloudData
    {
    public:
    using POINT =pcl::PointXYZ;
    using CLOUD =pcl::PointCloud<POINT>;
    using CLOUD_PTR =CLOUD::Ptr;//智能指针share_ptr,一个对象可以被多个share_ptr拥有 

    double time = 0.0;
    CLOUD_PTR cloud_ptr;

    CloudData():cloud_ptr(new CLOUD()){}//指针初始化。为哈后边要加括号？表示值初始化(也可以用{}表示)不加括号也行，类加与不加都会调用构造函数。
    //如果加括号，内建数据类型会进行值初始化（与缺省初始化不同），如（new int()）表示初始化为0
    };
}

#endif