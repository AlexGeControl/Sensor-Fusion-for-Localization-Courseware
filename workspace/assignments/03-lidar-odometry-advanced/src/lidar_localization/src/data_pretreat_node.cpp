/*
 * @Description: LOAM scan registration node
 * @Author: Ge Yao
 * @Date: 2021-05-02 12:56:27
 */
#include <memory>

#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_localization/saveMap.h>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_pretreat_node");
    ros::NodeHandle nh;

    // register front end processing workflow:
    std::unique_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_unique<DataPretreatFlow>(nh);

    // process rate: 10Hz
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();

        rate.sleep();
    }

    return EXIT_SUCCESS;
}