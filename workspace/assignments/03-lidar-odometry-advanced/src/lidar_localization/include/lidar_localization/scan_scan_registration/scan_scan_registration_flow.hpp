/*
 * @Description: LOAM scan-scan registration facade
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#ifndef LIDAR_LOCALIZATION_SCAN_SCAN_REGISTRATION_FLOW_HPP_
#define LIDAR_LOCALIZATION_SCAN_SCAN_REGISTRATION_FLOW_HPP_

#include <memory>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"

#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

#include "lidar_localization/scan_scan_registration/scan_scan_registration.hpp"

namespace lidar_localization {

class ScanScanRegistrationFlow {
  public:
    ScanScanRegistrationFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool InitParam(const YAML::Node& config_node);
    bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);
    bool InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node);

    bool ReadData(void);
    bool HasData(void);
    bool ValidData(void);
    bool UpdateData(void);
    bool PublishData(void);
    
  private:
    struct {
      int num_frames_skip{1};
    } config_;

    // whether the front end is inited:
    bool inited_{false};

    // inputs: registered scans
    std::unique_ptr<CloudSubscriber> filtered_cloud_sub_ptr_{nullptr};
    std::deque<CloudData> filtered_cloud_buff_;
    CloudData filtered_cloud_;

    std::unique_ptr<CloudSubscriber> corner_points_sharp_sub_ptr_{nullptr};
    std::deque<CloudData> corner_points_sharp_buff_;
    CloudData corner_points_sharp_;

    std::unique_ptr<CloudSubscriber> corner_points_less_sharp_sub_ptr_{nullptr};
    std::deque<CloudData> corner_points_less_sharp_buff_;
    CloudData corner_points_less_sharp_;

    std::unique_ptr<CloudSubscriber> surf_points_flat_sub_ptr_{nullptr};
    std::deque<CloudData> surf_points_flat_buff_;
    CloudData surf_points_flat_;

    std::unique_ptr<CloudSubscriber> surf_points_less_flat_sub_ptr_{nullptr};
    std::deque<CloudData> surf_points_less_flat_buff_;
    CloudData surf_points_less_flat_;

    // LOAM front end implementation:
    std::unique_ptr<ScanScanRegistration> scan_scan_registration_ptr_{nullptr};

    // outputs:
    std::unique_ptr<CloudPublisher> mapping_full_points_pub_ptr_{nullptr};
    std::unique_ptr<CloudPublisher> mapping_sharp_points_pub_ptr_{nullptr};
    std::unique_ptr<CloudPublisher> mapping_flat_points_pub_ptr_{nullptr};

    std::unique_ptr<OdometryPublisher> odom_scan_to_scan_pub_ptr_;
    Eigen::Matrix4f odometry_ = Eigen::Matrix4f::Identity();
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SCAN_SCAN_REGISTRATION_FLOW_HPP_