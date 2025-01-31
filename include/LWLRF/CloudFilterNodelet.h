#pragma once

#define PCL_NO_PRECOMPILE
#include "RadarPoint.h"
#include "RadarConfig.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <nav_ross/nav_msg.h>

namespace LWLRF {
    class CloudFilterNodelet : public nodelet::Nodelet {
        public:
          void onInit() override;

        private:
          void ConfigurationCallback(const nav_ross::nav_msgConstPtr& ConfigMsg);
          void DataCallback(const sensor_msgs::PointCloud2& input_cloud) const;

          // ROS Node Handles
          ros::NodeHandle m_NodeHandle_;
          ros::NodeHandle m_PrivateNodeHandle_;

          // ROS Subscribers
          ros::Subscriber m_PCLSubscriber_;
          ros::Subscriber m_ConfigurationSubscriber_;

          // ROS Publishers
          ros::Publisher m_PCLPublisher_;

          // ROS Parameters
          std::string m_paramPCLTopic;
          std::string m_paramConfigurationTopic;
          int m_paramDownsamplingFactor;
          int m_paramCFARGuardCells;
          int m_paramCFARTrainCells;

          // Variables
          RadarConfig m_RadarConfiguration_;
          pcl::PointCloud<LWLRF::PointRADAR>::Ptr m_InputPCL_;
          pcl::PointCloud<LWLRF::PointRADAR>::Ptr m_FilteredPCL_;
          int m_ScanSize;
    };
}