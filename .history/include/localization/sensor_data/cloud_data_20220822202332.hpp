/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 20:12:40
 * @LastEditTime: 2022-08-22 20:23:32
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
        using cloudPtr = pcl::PointCloud<pointType>::Ptr;

    public:
        CloudData();

    private:
        cloudPtr cloud_ptr_;
    };
}