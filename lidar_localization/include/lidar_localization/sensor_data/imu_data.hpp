#ifndef IMU_DATA_HPP
#define IMU_DATA_HPP

#include<Eigen/Dense>

namespace lidar_localization{
    class IMUData
    {
    public:
        struct LinearAcceleration
        {
            double x=0.0;
            double y=0.0;
            double z=0.0;
        };
        struct AugularVelocity
        {
            double x=0.0;
            double y=0.0;
            double z=0.0;
        };
        struct Orientation
        {
            //四元数表示旋转
            double x =0.0;
            double y=0.0;
            double z=0.0;
            double w =0.0;
        };
        
        double time =0.0;
        LinearAcceleration linear_acceleration;
        AugularVelocity augular_velocity;
        Orientation orientation;

        public:
        //将四元数旋转转化为旋转矩阵并返回
            Eigen::Matrix3f GetOrientationMatrix(){
                Eigen::Quaterniond  q(orientation.w, orientation.x, orientation.y, orientation.z);
                Eigen::Matrix3f matrix =q.matrix().cast<float>();//强制类型转换,不转换报错

                return matrix;
            }
        
    };
    
}

#endif