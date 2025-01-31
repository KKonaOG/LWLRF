#include "LWLRF/FFTConverterNodelet.h"
#include <pluginlib/class_list_macros.hpp>
#define SPEED_OF_LIGHT_IN_VACUUM_M_PER_NSEC 0.299792
PLUGINLIB_EXPORT_CLASS(LWLRF::FFTConverterNodelet, nodelet::Nodelet);

namespace LWLRF {
    void FFTConverterNodelet::FFTCallback(const nav_ross::HighPrecisionFFTDataConstPtr &HPFFTDataMsg) {
        const double theta = (static_cast<double>(HPFFTDataMsg->azimuth) / static_cast<double>(m_RadarConfiguration_.encoderSize) * 2 * M_PI);

        // Azimuth Creation
        for (size_t range_bin = 0; range_bin < HPFFTDataMsg->data.size(); range_bin++)
        {
            LWLRF::PointRADAR p{};
            
            /* Due to a PCL Issue where LWLRF::PointRADAR cannot contain a uint64_t for a nanosecond timestamp we store it as two uint32_t variables
             * // TODO: This is resolved in later versions and should be converted in higher versions of ROS / Ubuntu
             */
            const uint64_t point_time_nsec = HPFFTDataMsg->header.stamp.toNSec() - ((m_RadarConfiguration_.rangeResolution * HPFFTDataMsg->data.size() - m_RadarConfiguration_.rangeResolution * range_bin) / SPEED_OF_LIGHT_IN_VACUUM_M_PER_NSEC);
            const uint32_t point_time_high_nsec = point_time_nsec >> 32;
            const uint32_t point_time_low_nsec = point_time_nsec & 0xFFFFFFFF;

            p.x = m_RadarConfiguration_.rangeResolution * range_bin * cos(theta);
            p.y = m_RadarConfiguration_.rangeResolution * range_bin * sin(theta);
            p.z = 0;
            p.intensity = static_cast<uint8_t>(HPFFTDataMsg->data[range_bin] / 65535.0 * 255);
            p.azimuth = HPFFTDataMsg->azimuth;
            p.time_high = point_time_high_nsec;
            p.time_low = point_time_low_nsec;
            m_pAzimuthPCL_->points.push_back(p);
        }

        pcl_conversions::toPCL(HPFFTDataMsg->header.stamp, m_pAzimuthPCL_->header.stamp);
        m_pAzimuthPCL_->header.frame_id = m_paramSensorFrame;
        m_pAzimuthPCL_->header.seq = m_azimuthFrameNumber_;
        m_pAzimuthPCL_->height = 1;
        m_pAzimuthPCL_->width = m_pAzimuthPCL_->points.size();
        m_AzimuthPublisher_.publish(m_pAzimuthPCL_);


        if (HPFFTDataMsg->azimuth < m_lastAzimuth_)
        {
            m_pCompleteScanPCL_->header.frame_id = m_paramSensorFrame;
            m_pCompleteScanPCL_->header.seq = m_scanFrameNumber_;
            m_pCompleteScanPCL_->height = 1;
            m_pCompleteScanPCL_->width = m_pCompleteScanPCL_->points.size();
            m_PCLPublisher_.publish(m_pCompleteScanPCL_);
            m_pCompleteScanPCL_->clear();
        }

        *m_pCompleteScanPCL_ += *m_pAzimuthPCL_;
        // This is set here instead of in the if because the header stamp should be the last stamp associated with the cloud being published.
        m_pCompleteScanPCL_->header.stamp = m_pAzimuthPCL_->header.stamp;
        m_pAzimuthPCL_->clear();
        m_azimuthFrameNumber_++;
        m_lastAzimuth_ = HPFFTDataMsg->azimuth;
    };

    void FFTConverterNodelet::ConfigurationCallback(const nav_ross::nav_msgConstPtr &ConfigMsg) {
        // One call, thats all!
        /*
            This code flow can be a bit confusing, so as a quick breakdown.

            1. Subscribed in onInit to the m_paramConfigurationTopic, routing to this callback
            2. Callback is reached, and the subscriber is shutdown as we don't need it again.
            3. The configuration is copied from the recieved data.
            4. The FFT subscriber is stood up, this is done inside the ConfigurationCallback since
               we can't operate on the data without the ConfigurationData.

         */
        m_ConfigurationSubscriber_.shutdown();

        m_RadarConfiguration_.azimuthSamples = ConfigMsg->AzimuthSamples;
        m_RadarConfiguration_.binSize = ConfigMsg->BinSize;
        m_RadarConfiguration_.encoderSize = ConfigMsg->EncoderSize;
        m_RadarConfiguration_.expectedRotationRate = ConfigMsg->ExpectedRotationRate;
        m_RadarConfiguration_.rangeInBins = ConfigMsg->RangeInBins;
        m_RadarConfiguration_.rangeResolution = ConfigMsg->range_resolution;
        m_FFTSubscriber_ = m_NodeHandle_.subscribe(m_paramFFTDataTopic, 0, &FFTConverterNodelet::FFTCallback, this);
    };


    void FFTConverterNodelet::onInit() {
        m_NodeHandle_ = getMTNodeHandle();
        m_PrivateNodeHandle_ = getMTPrivateNodeHandle();

        NODELET_INFO_NAMED("[LWLRF]", "FFTConverter Started");
        m_PrivateNodeHandle_.getParam("fft_topic", m_paramFFTDataTopic);
        m_PrivateNodeHandle_.getParam("configuration_topic", m_paramConfigurationTopic);
        m_PrivateNodeHandle_.getParam("sensor_frame", m_paramSensorFrame);

        m_pAzimuthPCL_.reset(new pcl::PointCloud<LWLRF::PointRADAR>());
        m_pCompleteScanPCL_.reset(new pcl::PointCloud<LWLRF::PointRADAR>());

        m_AzimuthPublisher_ = m_PrivateNodeHandle_.advertise<sensor_msgs::PointCloud2>("raw_azimuth", 1);
        m_PCLPublisher_ = m_PrivateNodeHandle_.advertise<sensor_msgs::PointCloud2>("raw_scan", 1);
        m_ConfigurationSubscriber_ = m_NodeHandle_.subscribe(m_paramConfigurationTopic, 1, &FFTConverterNodelet::ConfigurationCallback, this);

        NODELET_INFO_NAMED("[LWLRF]", "FFTConverter Initialized");
    };
}