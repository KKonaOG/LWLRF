#pragma once

#define PCL_NO_PRECOMPILE // Needed anywhere we want to use our custom Point
#include "RadarConfig.h"
#include "RadarPoint.h"
#include <pcl/point_cloud.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_ross/HighPrecisionFFTData.h>
#include <nav_ross/nav_msg.h>

namespace LWLRF {
    class FFTConverterNodelet : public nodelet::Nodelet {
        public:
          void onInit() override;

        private:
          void ConfigurationCallback(const nav_ross::nav_msgConstPtr& ConfigMsg);
          void FFTCallback(const nav_ross::HighPrecisionFFTDataConstPtr& HPFFTDataMsg);
          void ScanCycleComplete();

          // ROS Node Handles
          ros::NodeHandle m_NodeHandle_;
          ros::NodeHandle m_PrivateNodeHandle_;

          // ROS Subscribers
          ros::Subscriber m_FFTSubscriber_;
          ros::Subscriber m_PCLSubscriber_;
          ros::Subscriber m_ConfigurationSubscriber_;

          // ROS Publishers
          ros::Publisher m_PCLPublisher_;
          ros::Publisher m_AzimuthPublisher_;

          // ROS Parameters
          std::string m_paramFFTDataTopic;
          std::string m_paramConfigurationTopic;
          std::string m_paramSensorFrame;

          // Variables
          long int m_azimuthFrameNumber_ = 0; // Frame Number for the PCL Message
          long int m_scanFrameNumber_ = 0;
          uint16_t m_lastAzimuth_ = 0; // Azimuth seen in previous call to FFTCallback
          RadarConfig m_RadarConfiguration_ = RadarConfig(); // Holds the RADAR configuration set by ConfigurationCallback
          pcl::PointCloud<LWLRF::PointRADAR>::Ptr  m_pAzimuthPCL_; // Utilize a shared pointer rather than allocating new memory each time we need a new PointCloud
          pcl::PointCloud<LWLRF::PointRADAR>::Ptr  m_pCompleteScanPCL_; // Utilize a shared pointer rather than allocating new memory each time we need a new PointCloud
    };
}