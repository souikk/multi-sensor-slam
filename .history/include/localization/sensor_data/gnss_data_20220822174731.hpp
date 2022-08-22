/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 17:20:07
 * @LastEditTime: 2022-08-22 17:47:30
 * @Description:
 */
#pragma once

#include <GeographicLib/LocalCartesian.hpp>

namespace localization
{
    class GNSSData
    {
    public:
        double time = 0.0;
        double longitude = 0.0;
        double latitude = 0.0;
        double altitude = 0.0;
        double local_E = 0.0;
        double local_N = 0.0;
        double local_U = 0.0;

        static double origin_longitude;
        static double origin_latitude;
        static double origin_altitude;

    private:
        static GeographicLib::LocalCartesian geo_converter;
        static bool origin_position_inited;

    public:
        void InitOriginPosition();
        void updateENU;
    };
} // namespace localization
