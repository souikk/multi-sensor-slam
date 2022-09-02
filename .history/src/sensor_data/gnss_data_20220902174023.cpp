/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 18:05:22
 * @LastEditTime: 2022-09-02 17:39:48
 * @Description:
 */
#include "sensor_data/gnss_data.hpp"

#include "glog/logging.h"

double localization::GnssData::origin_longitude = 0.0;
double localization::GnssData::origin_latitude = 0.0;
double localization::GnssData::origin_altitude = 0.0;
bool localization::GnssData::origin_position_inited = false;
GeographicLib::LocalCartesian localization::GnssData::geo_converter;

namespace localization
{
    void GnssData::initOriginPosition()
    {
        geo_converter.Reset(latitude, longitude, altitude);
        origin_latitude = latitude;
        origin_longitude = longitude;
        origin_altitude = altitude;

        origin_position_inited = true;
    }

    void GnssData::updateENU()
    {
        if (!origin_position_inited)
        {
            LOG(WARNING) << "GeoConverter has not set origin position";
        }
        geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
    }

    bool GnssData::syncPointCloud(double timestamp, std::deque<GnssData> gnss_buf, GnssData &sync_gnss_data)
    {
        //$ 判断点云时间戳是否在gnss队列时间戳范围内
        if (timestamp < gnss_buf.front().time || timestamp >= gnss_buf.back().time)
        {
            return false;
        }

        //$ 删除过早的gnss数据
        while (gnss_buf.size() >= 2)
        {

            if (timestamp >= gnss_buf.front().time && timestamp <= gnss_buf.at(1).time)
            {
                break;
            }
            gnss_buf.pop_front();
        }
        if (gnss_buf.size() < 2)
        {
            return false;
        }

        //$ 插值计算当前点云时刻的imu数值
        GnssData data_front = gnss_buf.at(0);
        GnssData data_back = gnss_buf.at(1);
        double diff = data_back.time - data_front.time;
        double front_scale = (data_back.time - timestamp) / diff;
        double back_scale = (timestamp - data_front.time) / diff;

        if (timestamp - data_front.time > 0.15 || data_back.time - timestamp > 0.15)
        {
            return false;
        }

        sync_gnss_data.longitude = front_scale * data_front.longitude + back_scale * data_back.longitude;
        sync_gnss_data.latitude = front_scale * data_front.latitude + back_scale * data_back.latitude;
        sync_gnss_data.altitude = front_scale * data_front.altitude + back_scale * data_back.altitude;

        Eigen::Quaterniond q0(data_front.orientation.w, data_front.orientation.x,
                              data_front.orientation.y, data_front.orientation.z);
        Eigen::Quaterniond q1(data_back.orientation.w, data_back.orientation.x,
                              data_back.orientation.y, data_back.orientation.z);
        Eigen::Quaterniond q = q0.slerp(back_scale, q1);
        sync_gnss_data.orientation.w = q.w();
        sync_gnss_data.orientation.x = q.x();
        sync_gnss_data.orientation.y = q.y();
        sync_gnss_data.orientation.z = q.z();
        sync_gnss_data.orientation.Normlize();

        return true;
    }
}