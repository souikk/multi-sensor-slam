/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 17:20:07
 * @LastEditTime: 2022-08-24 14:11:56
 * @Description:
 */
#pragma once
#include <deque>

#include <Eigen/Geometry>

#include "GeographicLib/LocalCartesian.hpp"

namespace localization
{
    class GnssData
    {
    public:
        double time = 0.0;
        double longitude = 0.0;
        double latitude = 0.0;
        double altitude = 0.0;
        double local_E = 0.0;
        double local_N = 0.0;
        double local_U = 0.0;

        class Orientation
        {
        public:
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 0.0;

        public:
            void Normlize()
            {
                double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
                x /= norm;
                y /= norm;
                z /= norm;
                w /= norm;
            }
        };

        Orientation orientation;
        static double origin_longitude;
        static double origin_latitude;
        static double origin_altitude;

    private:
        static bool origin_position_inited;
        static GeographicLib::LocalCartesian geo_converter;

    public:
        void initOriginPosition();
        void updateENU();

        static bool syncPointCloud(double timestamp, std::deque<GnssData> gnss_buf, GnssData &sync_gnss_data);
    };
} // namespace localization
