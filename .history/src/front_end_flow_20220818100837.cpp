/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 15:40:31
 * @LastEditTime: 2022-08-18 10:08:34
 * @Description:
 */
#include "front_end_flow.hpp"

namespace localization
{
    FrontEndFlow::FrontEndFlow() : Node("FrontEndNode")
    {
        subImu_ = std::make_share<localization::ImuSubscriber>()
    }
}