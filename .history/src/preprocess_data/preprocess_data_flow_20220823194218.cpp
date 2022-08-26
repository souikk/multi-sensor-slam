/*
 *                        ::
 *                       :;J7, :,                        ::;7:
 *                       ,ivYi, ,                       ;LLLFS:
 *                       :iv7Yi                       :7ri;j5PL
 *                      ,:ivYLvr                    ,ivrrirrY2X,
 *                      :;r@Wwz.7r:                :ivu@kexianli.
 *                     :iL7::,:::iiirii:ii;::::,,irvF7rvvLujL7ur
 *                    ri::,:,::i:iiiiiii:i:irrv177JX7rYXqZEkvv17
 *                 ;i:, , ::::iirrririi:i:::iiir2XXvii;L8OGJr71i
 *               :,, ,,:   ,::ir@mingyi.irii:i:::j1jri7ZBOS7ivv,
 *                  ,::,    ::rv77iiiriii:iii:i::,rvLq@huhao.Li
 *              ,,      ,, ,:ir7ir::,:::i;ir:::i:i::rSGGYri712:
 *            :::  ,v7r:: ::rrv77:, ,, ,:i7rrii:::::, ir7ri7Lri
 *           ,     2OBBOi,iiir;r::        ,irriiii::,, ,iv7Luur:
 *         ,,     i78MBBi,:,:::,:,  :7FSL: ,iriii:::i::,,:rLqXv::
 *         :      iuMMP: :,:::,:ii;2GY7OBB0viiii:i:iii:i:::iJqL;::
 *        ,     ::::i   ,,,,, ::LuBBu BBBBBErii:i:i:i:i:i:i:r77ii
 *       ,       :       , ,,:::rruBZ1MBBqi, :,,,:::,::::::iiriri:
 *      ,               ,,,,::::i:  @arqiao.       ,:,, ,:::ii;i7:
 *     :,       rjujLYLi   ,,:::::,:::::::::,,   ,:i,:,,,,,::i:iii
 *     ::      BBBBBBBBB0,    ,,::: , ,:::::: ,      ,,,, ,,:::::::
 *     i,  ,  ,8BMMBBBBBBi     ,,:,,     ,,, , ,   , , , :,::ii::i::
 *     :      iZMOMOMBBM2::::::::::,,,,     ,,,,,,:,,,::::i:irr:i:::,
 *     i   ,,:;u0MBMOG1L:::i::::::  ,,,::,   ,,, ::::::i:i:iirii:i:i:
 *     :    ,iuUuuXUkFu7i:iii:i:::, :,:,: ::::::::i:i:::::iirr7iiri::
 *     :     :rk@Yizero.i:::::, ,:ii:::::::i:::::i::,::::iirrriiiri::,
 *      :      5BMBBBBBBSr:,::rv2kuii:::iii::,:i:,, , ,,:,:i@petermu.,
 *           , :r50EZ8MBBBBGOBBBZP7::::i::,:::::,: :,:,::i;rrririiii::
 *               :jujYY7LS0ujJL7r::,::i::,::::::::::::::iirirrrrrrr:ii:
 *            ,:  :@kevensun.:,:,,,::::i:i:::::,,::::::iir;ii;7v77;ii;i,
 *            ,,,     ,,:,::::::i:iiiii:i::::,, ::::iiiir@xingjief.r;7:i,
 *         , , ,,,:,,::::::::iiiiiiiiii:,:,:::::::::iiir;ri7vL77rrirri::
 *          :,, , ::::::::i:::i:::i:i::,,,,,:,::i:i:::iir;@Secbone.ii:::
 */

/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 10:31:08
 * @LastEditTime: 2022-08-23 19:39:35
 * @Description: 数据预处理的工作流程
 */
#include "preprocess_data/preprocess_data_flow.hpp"

namespace localization
{

    PreprocessDataFlow::PreprocessDataFlow(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = node_ptr;
        std::string config_file_path = WORKSPACE_PATH + "/config/preprocess_data/config.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);

        initSubscriber(node_ptr_, config["measurements"]);
    }

    void PreprocessDataFlow::initSubscriber(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config)
    {
        subImu_ = std::make_shared<ImuSubscriber>(
            node_ptr,
            config["imu"]["topic_name"].as<std::string>(),
            config["imu"]["queue_size"].as<size_t>(),
            config["imu"]["policy_history"].as<int>());
        subLidar_ = std::make_shared<LidarSubscriber>(
            node_ptr,
            config["lidar"]["topic_name"].as<std::string>(),
            config["lidar"]["queue_size"].as<size_t>(),
            config["lidar"]["policy_history"].as<int>());
        subGnss_ = std::make_shared<GnssSubscriber>(
            node_ptr,
            config["gnss"]["topic_name"].as<std::string>(),
            config["gnss"]["queue_size"].as<size_t>(),
            config["gnss"]["policy_history"].as<int>());
    }
    /**
     * @description:
     * @return {*}
     */
    void PreprocessDataFlow::run()
    {
        if (!read())
        {
            return;
        }
    }

    bool PreprocessDataFlow::read()
    {
    }
}