/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {

template <typename T>
class CloudSubscriber {
  public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
      :nh_(nh) {
      subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber<T>::msg_callback, this);
    };

    CloudSubscriber() = default;
    void ParseData(std::deque<T>& cloud_data_buff) {
      buff_mutex_.lock();
      
      if (!new_cloud_data_.empty()) {
          std::copy(
              std::make_move_iterator(new_cloud_data_.begin()), 
              std::make_move_iterator(new_cloud_data_.end()), 
              std::back_inserter(cloud_data_buff)
          );

          new_cloud_data_.clear();
      }

      buff_mutex_.unlock();
    };

  private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
      buff_mutex_.lock();

      T cloud_data;
      cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
      pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

      new_cloud_data_.push_back(std::move(cloud_data));

      buff_mutex_.unlock();
    };

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<T> new_cloud_data_;

    std::mutex buff_mutex_;
};

}

#endif