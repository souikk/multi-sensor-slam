/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 20:12:40
 * @LastEditTime: 2022-08-22 20:21:25
 * @Description:
 */
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace localization
{
    class CloudData
    {
    public:
        using pointType = pcl::PointXYZI;
        using cloudType = pcl::PointCloud<pointType>;
        using cloudType_ptr = pcl::PointCloud<pointType>::Ptr;

    public:
        CloudData();
    };
}