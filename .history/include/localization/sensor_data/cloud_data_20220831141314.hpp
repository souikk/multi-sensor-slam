/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 20:12:40
 * @LastEditTime: 2022-08-31 14:13:14
 * @Description:
 */
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "cloud_type/cloud_type.hpp"
namespace localization
{
    class CloudData
    {
    public:
        using pointType = MyPointXYZICR;
        using cloudType = pcl::PointCloud<pointType>;
        using cloudPtr = pcl::PointCloud<pointType>::Ptr;

    public:
        CloudData() : cloud_ptr_(new cloudType())
        {
        }

    public:
        double time;
        cloudPtr cloud_ptr_;
    };
}