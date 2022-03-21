/*
tf监听
*/
#include "../../include/lidar_localization/tf_listener/tf_listener.hpp"

#include <Eigen/Geometry>

namespace lidar_localization
{
    TFListener::TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id)
    :nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id)
    {}
    bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix)
    {
        try{
            tf::StampedTransform transform;
            listener_.lookupTransform(base_frame_id_,child_frame_id_, ros::Time(0), transform);
            //lookupTransform获取tf数据，将转换关系储存到transform,得到source_frame到target_frame的坐标变换，即chilid_frame_id_到base_frame_id的坐标变换
            TransformToMatrix(transform, transform_matrix);//转换格式
            return true;
        }//如果throw tf::TransformException类型异常执行catch
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }
    }
    bool TFListener::TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix)
    {
        Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
        //Translationg3f<float,3> 表示平移？,得到source_frame到target_frame的坐标变换，即chilid_frame_id_到base_frame_id的坐标变换
        std::cout <<"X:"<<transform.getOrigin().getX()<<std::endl<<"Y:"<<transform.getOrigin().getY()<<std::endl<<"Z:"<<transform.getOrigin().getZ()<<std::endl;
        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);//getRotation()返回表示旋转的四元数,getEulerYPR由四元数求得欧拉角
        Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
        //Eigen::AngleAxisf旋转向量3*1，方向表示旋转方向，模长表示旋转角。Eigen::Vector3f::UnitX()x轴单位向量，rot_x_btol表示沿x单位向量为旋转轴，旋转roll弧度

        // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
       
        transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();//这个顺序与万向角问题有关，按航向->俯仰->横滚，此时万向锁出现在俯仰角为正负90度时，车辆等载体一般不可能出像这种情况
       //Matrix4f(4*4)和仿射变换矩阵Affine3f(4x4)之间的转换需要用matrix()函数
        return true;
    }
}