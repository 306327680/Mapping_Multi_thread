//
// Created by echo on 23-4-19.
//

#ifndef MAPPING_MULTI_THREAD_MAPPING_H
#define MAPPING_MULTI_THREAD_MAPPING_H
#include <pcl/io/pcd_io.h>//不能两次引用头文件
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/filter.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <ros/ros.h>

// point cloud define
struct PointXYZITR {
    PCL_ADD_POINT4D
    float intensity;
    float timestamp;
    float ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZITR,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)(float, timestamp, timestamp)(float, ring, ring))

typedef PointXYZITR mypcd;
typedef pcl::PointCloud<mypcd> mypcdCloud;

class mapping {
    mapping(){};
    mapping(int start,int end,int thread_num = 0){_start_id = start,_end_id = end,_thread_num = thread_num;}

public:
    int _start_id = 0;
    int _end_id = 0;
    int _thread_num = 0;

//1. read pcd and down sample
    void readPointCloud(std::string file_names,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai,
                                       ros::Time& cur_time,mypcdCloud& xyzItimeRing,std::string LiDAR_type);
//2. distortion
//3. matching
//4. publish result

};


#endif //MAPPING_MULTI_THREAD_MAPPING_H
