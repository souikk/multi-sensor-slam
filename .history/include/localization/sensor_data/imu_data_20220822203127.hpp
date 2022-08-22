/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 20:29:26
 * @LastEditTime: 2022-08-22 20:31:22
 * @Description:
 */
#pragma once
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
        };
    }
}