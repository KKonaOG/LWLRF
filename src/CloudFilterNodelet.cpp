#include "LWLRF/CloudFilterNodelet.h"

#include <pcl/common/centroid.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
PLUGINLIB_EXPORT_CLASS(LWLRF::CloudFilterNodelet, nodelet::Nodelet);


#define RADAR_INTENSITY_THRESHOLD 1.75

namespace LWLRF {
    void CloudFilterNodelet::DataCallback(const sensor_msgs::PointCloud2& input_cloud) const
    {
        if (input_cloud.width != m_ScanSize)
        {
            ROS_ERROR("Input cloud size must be equal to the size of scan");
            return;
        }

        pcl::fromROSMsg(input_cloud, *m_InputPCL_);


        // 2-D CA-CFAR (Designed for Full Scan)
        const int numRangeBinsPerAzimuth = m_RadarConfiguration_.rangeInBins;
        for (int azimuthOffset = 0; azimuthOffset < m_InputPCL_->width; azimuthOffset+=numRangeBinsPerAzimuth) {
            for (int range = 0; range < numRangeBinsPerAzimuth; range++) {
                int range_min_index = azimuthOffset;
                int range_max_index = azimuthOffset + numRangeBinsPerAzimuth;
                int point_index = azimuthOffset + range;
                if (point_index - m_paramCFARTrainCells - m_paramCFARGuardCells < range_min_index || point_index + m_paramCFARTrainCells + m_paramCFARGuardCells > range_max_index) {
                    continue;
                }

                if (point_index - numRangeBinsPerAzimuth*(m_paramCFARTrainCells+m_paramCFARGuardCells) < 0 || point_index + numRangeBinsPerAzimuth*(m_paramCFARTrainCells+m_paramCFARGuardCells) > m_InputPCL_->width) {
                    continue;
                }

                float noise_sum = 0.0f;
                for (int offset = 0; offset < m_paramCFARTrainCells / 2; offset++) {
                    const int offset_reverse = m_paramCFARTrainCells - offset;
                    const int offset_forward = offset;

                    // Range CFAR
                    noise_sum += m_InputPCL_->points[point_index + offset_reverse - m_paramCFARGuardCells].intensity;
                    noise_sum += m_InputPCL_->points[point_index + offset_forward - m_paramCFARGuardCells].intensity;
                    noise_sum += m_InputPCL_->points[point_index - offset_reverse - m_paramCFARGuardCells].intensity;
                    noise_sum += m_InputPCL_->points[point_index - offset_forward - m_paramCFARGuardCells].intensity;

                    // Azimuth CFAR
                    noise_sum += m_InputPCL_->points[point_index - numRangeBinsPerAzimuth*(offset_reverse + m_paramCFARGuardCells)].intensity;
                    noise_sum += m_InputPCL_->points[point_index - numRangeBinsPerAzimuth*(offset_forward + m_paramCFARGuardCells)].intensity;
                    noise_sum += m_InputPCL_->points[point_index + numRangeBinsPerAzimuth*(offset_reverse + m_paramCFARGuardCells)].intensity;
                    noise_sum += m_InputPCL_->points[point_index + numRangeBinsPerAzimuth*(offset_forward + m_paramCFARGuardCells)].intensity;
                }

                const float noise_average = noise_sum / (static_cast<float>(m_paramCFARTrainCells) * 4.0f);
                // Update intensity directly in the downsampled cloud
                if (m_InputPCL_->points[point_index].intensity > (noise_average * RADAR_INTENSITY_THRESHOLD)) {
                    m_InputPCL_->points[point_index].intensity = 0.0f;
                }
            }
        }

        pcl::PassThrough<LWLRF::PointRADAR> passthroughFilter;
        passthroughFilter.setInputCloud(m_InputPCL_);
        passthroughFilter.setFilterFieldName("intensity");
        passthroughFilter.setFilterLimits(100, 255);
        passthroughFilter.filter(*m_FilteredPCL_);

        m_PCLPublisher_.publish(m_FilteredPCL_);
        m_FilteredPCL_->clear();
    };

    void CloudFilterNodelet::ConfigurationCallback(const nav_ross::nav_msgConstPtr &ConfigMsg) {
        // One call, thats all!
        /*
            This code flow can be a bit confusing, so as a quick breakdown.

            1. Subscribed in onInit to the m_paramConfigurationTopic, routing to this callback
            2. Callback is reached, and the subscriber is shutdown as we don't need it again.
            3. The configuration is copied from the recieved data.
         */
        m_ConfigurationSubscriber_.shutdown();

        m_RadarConfiguration_.azimuthSamples = ConfigMsg->AzimuthSamples;
        m_RadarConfiguration_.binSize = ConfigMsg->BinSize;
        m_RadarConfiguration_.encoderSize = ConfigMsg->EncoderSize;
        m_RadarConfiguration_.expectedRotationRate = ConfigMsg->ExpectedRotationRate;
        m_RadarConfiguration_.rangeInBins = ConfigMsg->RangeInBins;
        m_RadarConfiguration_.rangeResolution = ConfigMsg->range_resolution;


        // This is done here to prevent the need to calculate the multiplication every time data is received
        m_ScanSize = m_RadarConfiguration_.rangeInBins * m_RadarConfiguration_.azimuthSamples;

        m_PCLSubscriber_ = m_NodeHandle_.subscribe(m_paramPCLTopic, 1, &CloudFilterNodelet::DataCallback, this);
    };

    void CloudFilterNodelet::onInit() {
        m_NodeHandle_ = getMTNodeHandle();
        m_PrivateNodeHandle_ = getMTPrivateNodeHandle();

        NODELET_INFO_NAMED("[LWLRF]", "CloudFilter Started");

        m_PrivateNodeHandle_.getParam("input_pcl_topic", m_paramPCLTopic);
        m_PrivateNodeHandle_.getParam("configuration_topic", m_paramConfigurationTopic);
        m_PrivateNodeHandle_.getParam("downsampling_factor", m_paramDownsamplingFactor);
        m_PrivateNodeHandle_.getParam("cfar_guard_cells", m_paramCFARGuardCells);
        m_PrivateNodeHandle_.getParam("cfar_train_cells", m_paramCFARTrainCells);

        m_InputPCL_.reset(new pcl::PointCloud<LWLRF::PointRADAR>());
        m_FilteredPCL_.reset(new pcl::PointCloud<LWLRF::PointRADAR>());
        m_PCLPublisher_ = m_PrivateNodeHandle_.advertise<sensor_msgs::PointCloud2>("filtered_points", 1);
        m_ConfigurationSubscriber_ = m_NodeHandle_.subscribe(m_paramConfigurationTopic, 1, &CloudFilterNodelet::ConfigurationCallback, this);


        NODELET_INFO_NAMED("[LWLRF]", "CloudFilter Initialized");
    };
}