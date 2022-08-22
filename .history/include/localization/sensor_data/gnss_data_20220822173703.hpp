/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 17:20:07
 * @LastEditTime: 2022-08-22 17:37:02
 * @Description:
 */
#pragma once

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
    };
} // namespace localization
