#pragma once
#include <pcl/point_types.h>

namespace LWLRF
{
    struct EIGEN_ALIGN32 PointRADAR
    {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t time_high; // uint64_t can't be registered
        uint32_t time_low;
        uint16_t azimuth;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
}


POINT_CLOUD_REGISTER_POINT_STRUCT(LWLRF::PointRADAR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, time_high, time_high)
    (std::uint32_t, time_low, time_low)
    (std::uint16_t, azimuth, azimuth)
)
