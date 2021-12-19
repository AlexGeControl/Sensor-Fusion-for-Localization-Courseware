/*
 * @Description: LOAM data pre-processing interface
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_HPP_

#include <memory>

#include <vector>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

#include <pcl/filters/voxel_grid.h>


namespace lidar_localization {

enum FeaturePoint {
  CORNER_SHARP = 0,
  CORNER_LESS_SHARP = 1,
  SURF_FLAT = 2,
  SURF_LESS_FLAT = 3,
  NUM_TYPES = 4
};

class DataPretreat {
  public:
    static constexpr int kScanIndexSize = 64;
    static constexpr int kPointIndexSize = 400000;

    DataPretreat(void);

    bool Update(
      const CloudDataXYZ& input_cloud, 
      CloudDataXYZI::CLOUD_PTR &output_cloud,
      CloudDataXYZI::CLOUD_PTR &corner_sharp,
      CloudDataXYZI::CLOUD_PTR &corner_less_sharp,
      CloudDataXYZI::CLOUD_PTR &surf_flat,
      CloudDataXYZI::CLOUD_PTR &surf_less_flat
    );
  
  private:
    bool InitParam(const YAML::Node& config_node);
    bool InitFilters(const YAML::Node& config_node);

    bool FilterByRange(const CloudDataXYZ::CLOUD &input_cloud, CloudDataXYZ::CLOUD &output_cloud);

    bool GetScanId(const float &angle, int &scan_id);
    float GetCurvature(const CloudDataXYZI::CLOUD &cloud, int point_index);

    bool SortPointCloudByScan(const CloudDataXYZ::CLOUD &input_cloud, CloudDataXYZI::CLOUD &output_cloud);

    bool PickInNeighborhood(const CloudDataXYZI::CLOUD &cloud, const int point_index, const float thresh);
    bool GetFeaturePoints(
      const CloudDataXYZI::CLOUD &cloud, 
      CloudDataXYZI::CLOUD_PTR &corner_sharp,
      CloudDataXYZI::CLOUD_PTR &corner_less_sharp,
      CloudDataXYZI::CLOUD_PTR &surf_flat,
      CloudDataXYZI::CLOUD_PTR &surf_less_flat
    );

  private:
    struct {
      float scan_period;
      int num_scans;
      float min_range;
      int neighborhood_size;
      int num_sectors;
    } config_;

    struct {
      struct {
        int start[kScanIndexSize];
        int end[kScanIndexSize];
      } scan;

      struct {
        float curvature[kPointIndexSize];
        int index[kPointIndexSize];
        int picked[kPointIndexSize];
        int label[kPointIndexSize];
      } point;
    } index_;

    std::unique_ptr<pcl::VoxelGrid<CloudDataXYZI::POINT>> surf_less_flat_filter_ptr_{nullptr};
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_DATA_PRETREAT_HPP_