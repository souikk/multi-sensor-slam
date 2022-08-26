/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 20:29:26
 * @LastEditTime: 2022-08-24 10:50:17
 * @Description:
 */
#pragma once
#include <cmath>

namespace localization
{
    class ImuData
    {
    public:
        struct LinearAcceleration
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct AngularVelocity
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

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

        double time;
        LinearAcceleration linear_accleration;
        AngularVelocity angular_velocity;
        Orientation orientation;

    private:
        bool syncPointCloud(double timestamp, std::deque<ImuData> &imu_buf);
    };
}