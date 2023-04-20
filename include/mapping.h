//
// Created by echo on 23-4-19.
//

#ifndef MAPPING_MULTI_THREAD_MAPPING_H
#define MAPPING_MULTI_THREAD_MAPPING_H
#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>//不能两次引用头文件
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <ros/ros.h>
#include "pointTypes.h"
#include "g2oIO/PoseGraphIO.h"
#include "registration/registration.h"
#include <chrono>

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Isometry3d)
class mapping {
public:
    mapping(){};
    void init_mapping(std::vector<std::string> file_names,int thread_num = 0){_file_names = file_names,_thread_num = thread_num;};
    void start_mapping();
    std::vector<std::string> _file_names;
    int _thread_num = 0;
    //1. read pcd and down sample
    void readPointCloud(std::string file_names,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai,
                                       ros::Time& cur_time,mypcdCloud& xyzItimeRing,std::string LiDAR_type);
    //2. distortion
    //3. matching
    //4. publish result
private:
    const int N_SCAN = 16;
    bool _first_cloud = true;
    PoseGraphIO g2osaver;
    std::vector<Eigen::Matrix4f> poses;
    registration icp;

    //线性去畸变的接口
    void simpleDistortion(mypcdCloud input, Eigen::Matrix4f increase, pcl::PointCloud<pcl::PointXYZI> &output);
    void adjustDistortion(pcl::PointCloud<PointTypeBeam>::Ptr pointIn,pcl::PointCloud<PointTypeBeam>::Ptr &pointOut,
                                                  Eigen::Isometry3d transform);
    void trinterp(Eigen::Matrix4d &T0, Eigen::Matrix4d &T1, double r, Eigen::Matrix4d &T);
    void rotMat2quaternion(Eigen::Matrix4d &T, Eigen::Vector4d &q_quaternion);
    void qinterp(Eigen::Vector4d &Q1, Eigen::Vector4d &Q2, double r,
                          Eigen::Vector4d &q_quaternion_interpolation) ;
    void quatern2rotMat(Eigen::Vector4d &q_quaternion, Eigen::Matrix3d &R);
};


#endif //MAPPING_MULTI_THREAD_MAPPING_H
