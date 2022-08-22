/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 18:05:22
 * @LastEditTime: 2022-08-22 18:17:10
 * @Description:
 */
#include "sensor_data/gnss_data.hpp"

double localization::GNSSData::origin_longitude = 0.0;
double localization::GNSSData::origin_latitude = 0.0;
double localization::GNSSData::origin_altitude = 0.0;
bool localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian localization::GNSSData::geo_converter;
namespace localization
{
    void GNSSData::initOriginPosition()
    {
    }
}