/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 17:20:07
 * @LastEditTime: 2022-08-22 17:35:02
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
    };
} // namespace localization
