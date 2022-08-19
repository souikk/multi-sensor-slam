/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 15:40:31
 * @LastEditTime: 2022-08-19 16:19:06
 * @Description:
 */
#include "front_end_node.hpp"

namespace localization
{
    FrontEndFlow::FrontEndFlow() : Node("FrontEndNode")
    {
        subImu_ = std::make_shared<localization::ImuSubscriber>(this);
    }
}