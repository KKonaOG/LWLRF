/*
 * This implementation heavily mirrors filter_library/DeskewTF
 * In it's original implementation they were not compatible but should now generally be able to interoperate for each other.
 * Original DeskewTF may replace this code in the future.
 * DeskewTF Here: https://public.git.erdc.dren.mil/erdc_robotics/ros/utilities/filter_library/-/blob/main/pointcloud2_filters/src/filters/DeskewTF.cpp?ref_type=heads
*/

#include "LWLRF/MotionCompensationNodelet.h"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(LWLRF::MotionCompensationNodelet, nodelet::Nodelet);

namespace LWLRF {
    void MotionCompensationNodelet::DataCallback(const sensor_msgs::PointCloud2& inputCloud) const
    {
        sensor_msgs::PointCloud2 outputCloud = inputCloud;

        // TODO: This is different than DeskewTF as FFTConverter Nodelet sets stamp time to HPFFTDataMsg stamp (the time the data was received)
        ros::Time startTime = inputCloud.header.stamp - ros::Duration(1.0f / m_paramExpectedInputFrequency_);
        ros::Time endTime = inputCloud.header.stamp;

        try
        {
            if (m_tfBuffer_.canTransform(m_paramTargetFrameId_, inputCloud.header.frame_id, endTime, ros::Duration(m_paramExpectedInputFrequency_))) {
                geometry_msgs::TransformStamped startTransform = m_tfBuffer_.lookupTransform(m_paramTargetFrameId_, inputCloud.header.frame_id, startTime);
                geometry_msgs::TransformStamped endTransform = m_tfBuffer_.lookupTransform(m_paramTargetFrameId_, inputCloud.header.frame_id, endTime);

                // Iterators
                sensor_msgs::PointCloud2Iterator<float> it_x(outputCloud, "x");
                sensor_msgs::PointCloud2Iterator<float> it_y(outputCloud, "y");
                sensor_msgs::PointCloud2Iterator<float> it_z(outputCloud, "z");

                // TODO: This is different than DeskewTF as we use nanosecond accuracy to ensure individual points on an azimuth are able to be deskewed.
                sensor_msgs::PointCloud2Iterator<uint32_t> it_time_high(outputCloud, "time_high");
                sensor_msgs::PointCloud2Iterator<uint32_t> it_time_low(outputCloud, "time_low");

                for (; it_x != it_x.end() && it_y != it_y.end() && it_z != it_z.end(); ++it_x, ++it_y, ++it_z, ++it_time_high, ++it_time_low)
                {
                    geometry_msgs::PointStamped pt_original, pt_transformed;
                    pt_original.point.x = *it_x;
                    pt_original.point.y = *it_y;
                    pt_original.point.z = *it_z;

                    // Reconstructs timestamp from two uint32_ts representing an uint64_t nanosecond epoch time.
                    ros::Time timestamp = ros::Time().fromNSec((static_cast<uint64_t>(*it_time_high) << 32) | *it_time_high);

                    // TODO: This differs from DeskewTF due to the storage of the point's actual time in the cloud fields. This is critical since a points' timeshift from the output header time is not easily calculable when utilizing scans as an input.
                    pt_original.header.stamp = timestamp;

                    geometry_msgs::TransformStamped transform;
                    transform = interpolateTransform(startTransform, endTransform, timestamp);
                    tf2::doTransform(pt_original, pt_transformed, transform);
                    *it_x = pt_transformed.point.x;
                    *it_y = pt_transformed.point.y;
                    *it_z = pt_transformed.point.z;
                }

                geometry_msgs::TransformStamped transform;
                transform = m_tfBuffer_.lookupTransform(outputCloud.header.frame_id, m_paramTargetFrameId_, outputCloud.header.stamp);
                tf2::doTransform(outputCloud, outputCloud, transform);
            } else {
                NODELET_WARN_NAMED("[LWLRF]", "Motion Compensation is waiting for the transform from target frame (%s) to sensor frame (%s) to become available.", m_paramTargetFrameId_.c_str(), inputCloud.header.frame_id.c_str());
            }
        } catch (tf2::ExtrapolationException &ex)
        {
            NODELET_WARN_NAMED("[LWLRF]", "Motion Compensation unable to extrapolate transform from target frame (%s) to sensor frame (%s). Details: %s", m_paramTargetFrameId_.c_str(), inputCloud.header.frame_id.c_str(), ex.what());
            outputCloud = inputCloud;
        }

        m_OutputCloudPublisher_.publish(outputCloud);
    };

    geometry_msgs::TransformStamped MotionCompensationNodelet::interpolateTransform(const geometry_msgs::TransformStamped &one, const geometry_msgs::TransformStamped &two, const ros::Time time)
    {
        geometry_msgs::TransformStamped output;

        if (two.header.stamp == one.header.stamp)
        {
            output = two;
            return output;
        }

        // Begin Interpolation
        tf2Scalar ratio = (time - one.header.stamp).toSec() / (two.header.stamp - one.header.stamp).toSec();

        // Interpolate Transform
        tf2::Vector3 translation, translation1, translation2;
        tf2::convert(one.transform.translation, translation1);
        tf2::convert(two.transform.translation, translation2);
        translation.setInterpolate3(translation1, translation2, ratio);
        tf2::convert(translation, output.transform.translation);

        // Interpolate Rotation
        tf2::Quaternion rotation, rotation1, rotation2;
        tf2::convert(one.transform.rotation, rotation1);
        tf2::convert(two.transform.rotation, rotation2);
        rotation = tf2::slerp(rotation1, rotation2, ratio);
        tf2::convert(rotation, output.transform.rotation);

        output.header.stamp = time;
        output.header.frame_id = one.header.frame_id;
        output.child_frame_id = one.child_frame_id;

        return output;
    }

    MotionCompensationNodelet::MotionCompensationNodelet() : m_tfBuffer_(ros::Duration(60)), m_tfListener_(m_tfBuffer_)
    {
        NODELET_INFO_NAMED("[LWLRF]", "MotionCompensation Created");
    }


    void MotionCompensationNodelet::onInit()
    {
        NODELET_INFO_NAMED("[LWLRF]", "MotionCompensation Started");
        m_NodeHandle_ = getMTNodeHandle();
        m_PrivateNodeHandle_ = getMTPrivateNodeHandle();

        m_PrivateNodeHandle_.getParam("input_topic", m_paramInputTopic_);
        m_PrivateNodeHandle_.getParam("output_topic", m_paramOutputTopic_);
        m_PrivateNodeHandle_.getParam("target_frame_id", m_paramTargetFrameId_);
        m_PrivateNodeHandle_.getParam("expected_input_frequency", m_paramExpectedInputFrequency_);
        m_PrivateNodeHandle_.getParam("tf_lookup_timeout", m_paramTFWaitDelay_);
        m_PrivateNodeHandle_.getParam("output_queue_size", m_paramQueueSize_);

        NODELET_INFO_NAMED("[LWLRF]", "MotionCompensation Parameters Configured");

        m_InputCloudSubscriber_ = m_NodeHandle_.subscribe(m_paramInputTopic_, 0, &MotionCompensationNodelet::DataCallback, this);
        m_OutputCloudPublisher_ = m_PrivateNodeHandle_.advertise<sensor_msgs::PointCloud2>(m_paramOutputTopic_, m_paramQueueSize_, false);

        NODELET_INFO_NAMED("[LWLRF]", "MotionCompensation Initialized");
    };
}