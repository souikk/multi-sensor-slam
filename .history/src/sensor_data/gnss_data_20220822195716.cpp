/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 18:05:22
 * @LastEditTime: 2022-08-22 19:56:22
 * @Description:
 */
#include "sensor_data/gnss_data.hpp"

#include "glog/logging.h"

double localization::GNSSData::origin_longitude = 0.0;
double localization::GNSSData::origin_latitude = 0.0;
double localization::GNSSData::origin_altitude = 0.0;
bool localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian localization::GNSSData::geo_converter;

namespace localization
{
    void GNSSData::initOriginPosition()
    {
        geo_converter.Reset(latitude, longitude, altitude);
        origin_latitude = latitude;
        origin_longitude = longitude;
        origin_altitude = altitude;

        origin_position_inited = true;
    }

    void GNSSData::updateENU()
    {
        if (!origin_position_inited)
        {
            LOG(WARNING) << "GeoConverter has not set origin position";
        }
        geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
    }

}