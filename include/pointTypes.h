//
// Created by echo on 23-4-19.
//

#ifndef MAPPING_MULTI_THREAD_POINTTYPES_H
#define MAPPING_MULTI_THREAD_POINTTYPES_H
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

struct PointXYZIBT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float beam;
    float pctime;
    float range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIBT,
                                   (float, x, x) (float, y, y)
                                           (float, z, z) (float, intensity, intensity)
                                           (float, beam, beam)
                                           (float, pctime, pctime)
                                           (float, range, range)
)

typedef PointXYZIBT  PointTypeBeam;
//点云类型2 光滑度的
struct PointXYZIBS
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float beam;
    float pctime;
    float pointType;
    float range;
    float smooth;
    float NeighborPicked;
    float curvature;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIBS,
                                   (float, x, x) (float, y, y)
                                           (float, z, z) (float, intensity, intensity)
                                           (float, beam, beam)
                                           (float, pctime, pctime)
                                           (float, pointType, pointType)
                                           (float, range, range)
                                           (float, smooth, smooth)
                                           (float, NeighborPicked, NeighborPicked)
                                           (float, curvature, curvature)
)

typedef PointXYZIBS  PointTypeSm;

//todo  my pcd

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

//0.1. hesai LiDAR
namespace pandar_pointcloud
{
    struct PointXYZIT {
        PCL_ADD_POINT4D
        float intensity;
        double timestamp;
        uint16_t ring;                      ///< laser ring number
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
    } EIGEN_ALIGN16;

};

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

typedef pandar_pointcloud::PointXYZIT PPoint;
typedef pcl::PointCloud<PPoint> PPointCloud;
//0.2 vlp16


struct PointXYZIR {
    PCL_ADD_POINT4D
    float intensity;
    float time;
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;



POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZIR,
                                   (float, x, x)(float, y, y)(float, z, z)
                                           (float, intensity, intensity)(float, time, time)(uint16_t, ring, ring))
typedef  PointXYZIR VLPPoint;
typedef pcl::PointCloud<VLPPoint> VLPPointCloud;

//0.3 robosense
struct PointRobo {
    PCL_ADD_POINT4D
    float intensity;
    float time;
    float ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT( PointRobo,
                                   (float, x, x)(float, y, y)(float, z, z)
                                           (float, intensity, intensity)(float, time, time)(float, ring, ring))
typedef  PointRobo RoboPoint;
typedef pcl::PointCloud<RoboPoint> RoboPointCLoud;
//0.4 xyzirgb

struct PointXYZRGBI {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZRGBI,
                                   (float, x, x)(float, y, y)(float, z, z)
                                           (float, intensity, intensity) (std::uint32_t, rgba, rgba))

namespace ouster_ros {

    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                          (std::uint32_t, t, t)
                                          (std::uint16_t, reflectivity, reflectivity)
                                          (std::uint8_t, ring, ring)
                                          (std::uint16_t, ambient, ambient)
                                          (std::uint32_t, range, range)
)

#endif //MAPPING_MULTI_THREAD_POINTTYPES_H
