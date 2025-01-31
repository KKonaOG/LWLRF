#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace LWLRF {
    class MotionCompensationNodelet final : public nodelet::Nodelet {
        public:
            MotionCompensationNodelet();
            void onInit() override;

        private:
            static geometry_msgs::TransformStamped interpolateTransform(const geometry_msgs::TransformStamped &one, const geometry_msgs::TransformStamped &two, const ros::Time time);
            void DataCallback(const sensor_msgs::PointCloud2& inputCloud) const;


            // ROS Node Handles
            ros::NodeHandle m_NodeHandle_;
            ros::NodeHandle m_PrivateNodeHandle_;

            // ROS Subscribers
            ros::Subscriber m_InputCloudSubscriber_;

            // ROS Publishers
            ros::Publisher m_OutputCloudPublisher_;

            // ROS Parameters
            std::string m_paramInputTopic_;
            std::string m_paramOutputTopic_;
            std::string m_paramTargetFrameId_;
            float m_paramExpectedInputFrequency_;
            float m_paramTFWaitDelay_;
            int m_paramQueueSize_;

            // Variables
            tf2_ros::Buffer m_tfBuffer_;
            tf2_ros::TransformListener m_tfListener_;
    };
}