/*
 * @Description: LOAM scan-map registration node
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#include <memory>

#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_localization/saveMap.h>
#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/scan_map_registration/scan_map_registration_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "scan_map_registration_node");
    ros::NodeHandle nh;

    // register back end processing workflow:
    std::unique_ptr<ScanMapRegistrationFlow> scan_map_registration_flow_ptr = std::make_unique<ScanMapRegistrationFlow>(nh);

    // process rate: 3Hz
    while (ros::ok()) {
        ros::spinOnce();

        scan_map_registration_flow_ptr->Run();
    }

    return EXIT_SUCCESS;
}