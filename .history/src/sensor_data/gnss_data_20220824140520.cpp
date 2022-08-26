/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 18:05:22
 * @LastEditTime: 2022-08-24 14:05:20
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
        if (timestamp < gnss_buf.front().time)
        {
            return false;
        }
        while (gnss_buf.size() >= 2 && timestamp >= gnss_buf.at(1).time)
        {
            gnss_buf.pop_front();
        }
        if (gnss_buf.size() < 2)
        {
            return false;
        }

        if (timestamp == gnss_buf.front().time)
        {
            sync_gnss_data = gnss_buf.front();
            return true;
        }

        GnssData data_front = gnss_buf.at(0);
        GnssData data_back = gnss_buf.at(1);
        double diff = data_back.time - data_front.time;
        double front_scale = (data_back.time - timestamp) / diff;
        double back_scale = (timestamp - data_front.time) / diff;
    }
}