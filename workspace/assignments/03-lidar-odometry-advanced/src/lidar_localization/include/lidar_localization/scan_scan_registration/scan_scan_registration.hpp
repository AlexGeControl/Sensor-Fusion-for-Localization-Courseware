/*
 * @Description: LOAM scan-scan registration interface
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#ifndef LIDAR_LOCALIZATION_SCAN_SCAN_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_SCAN_SCAN_REGISTRATION_HPP_

#include <memory>

#include <vector>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "lidar_localization/models/loam/aloam_registration.hpp"

namespace lidar_localization {

class ScanScanRegistration {
  public:
    ScanScanRegistration(void);

    bool Update(
      CloudDataXYZI::CLOUD_PTR corner_sharp,
      CloudDataXYZI::CLOUD_PTR corner_less_sharp,
      CloudDataXYZI::CLOUD_PTR surf_flat,
      CloudDataXYZI::CLOUD_PTR surf_less_flat,
      Eigen::Matrix4f& lidar_odometry
    );

  private:
    struct CornerPointAssociation {
      int query_index{-1};

      double ratio{1.0};

      int associated_x_index{-1};
      int associated_y_index{-1};

      inline bool IsValid(void) const {
        return (query_index >= 0) && (associated_x_index >= 0) && (associated_y_index >= 0);
      }
    };

    struct SurfacePointAssociation {
      int query_index{-1};

      double ratio{1.0};
      
      int associated_x_index{-1};
      int associated_y_index{-1};
      int associated_z_index{-1};

      inline bool IsValid(void) const {
        return (query_index >= 0) && (associated_x_index >= 0) && (associated_y_index >= 0) && (associated_z_index >= 0);
      }
    };

  private:
    bool InitParam(const YAML::Node& config_node);
    bool InitKdTrees(void);

    bool TransformToStart(const CloudDataXYZI::POINT &input, CloudDataXYZI::POINT &output);

    bool AssociateCornerPoints(
      const CloudDataXYZI::CLOUD &corner_sharp,
      std::vector<CornerPointAssociation> &corner_point_associations
    );
    bool AssociateSurfacePoints(
      const CloudDataXYZI::CLOUD &surf_flat,
      std::vector<SurfacePointAssociation> &surface_point_associations
    );

    int AddEdgeFactors(
        const CloudDataXYZI::CLOUD &corner_sharp,
        const std::vector<CornerPointAssociation> &corner_point_associations,
        CeresALOAMRegistration &aloam_registration
    );
    int AddPlaneFactors(
        const CloudDataXYZI::CLOUD &surf_flat,
        const std::vector<SurfacePointAssociation> &surface_point_associations,
        CeresALOAMRegistration &aloam_registration
    );

    bool SetTargetPoints(
      const CloudDataXYZI::CLOUD_PTR &corner_less_sharp,
      const CloudDataXYZI::CLOUD_PTR &surf_less_flat
    );

    bool UpdateOdometry(Eigen::Matrix4f& lidar_odometry);

  private:
    struct {
      double scan_period{0.10};
      
      double distance_thresh{25.0};
      double scan_thresh{2.50};

      int num_threads{4};
      int max_num_iteration{4};
      double max_solver_time{0.05};
      CeresALOAMRegistration::Config registration_config;
    } config_;

    // target corner & plane feature points:
    struct {
      CloudDataXYZI::CLOUD_PTR candidate_corner_ptr;
      pcl::KdTreeFLANN<CloudDataXYZI::POINT>::Ptr corner;

      CloudDataXYZI::CLOUD_PTR candidate_surface_ptr;
      pcl::KdTreeFLANN<CloudDataXYZI::POINT>::Ptr surface;
    } kdtree_;

    // whether the front end is inited:
    bool inited_{false};

    // relative pose:
    Eigen::Quaterniond dq_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d dt_ = Eigen::Vector3d::Zero();

    // odometry:
    Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t_ = Eigen::Vector3d::Zero();
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SCAN_SCAN_REGISTRATION_HPP_