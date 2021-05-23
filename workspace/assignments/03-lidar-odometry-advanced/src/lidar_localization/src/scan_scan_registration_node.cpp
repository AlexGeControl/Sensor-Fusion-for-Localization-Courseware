/*
 * @Description: LOAM scan-scan registration node
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#include <memory>

#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_localization/saveMap.h>
#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/scan_scan_registration/scan_scan_registration_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "scan_scan_registration_node");
    ros::NodeHandle nh;

    // register front end processing workflow:
    std::unique_ptr<ScanScanRegistrationFlow> scan_scan_registration_ptr = std::make_unique<ScanScanRegistrationFlow>(nh);

    // process rate: 10Hz
    while (ros::ok()) {
        ros::spinOnce();

        scan_scan_registration_ptr->Run();
    }

    return EXIT_SUCCESS;
}