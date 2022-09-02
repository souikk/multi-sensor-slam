/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 09:50:10
 * @LastEditTime: 2022-08-31 14:12:02
 * @Description:
 */
#pragma once

// #include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct MyPointXYZICR
{
    PCL_ADD_POINT4D
    uint8_t intensity;
    uint16_t col;                   ///< laser column number
    uint16_t ring;                  ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointXYZICR,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(uint16_t, col, col))

// pandar128线
struct PandarPointXYZIRT
{
    PCL_ADD_POINT4D
    uint8_t intensity;
    double timestamp;               //每个点云的绝对时间
    uint16_t ring;                  ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment
POINT_CLOUD_REGISTER_POINT_STRUCT(PandarPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))
