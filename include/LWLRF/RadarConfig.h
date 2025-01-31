#pragma once

#include <cstdint>

namespace LWLRF {
    struct RadarConfig {
        double rangeResolution;
        uint16_t azimuthSamples;
        int encoderSize;
        int binSize;
        int rangeInBins;
        int expectedRotationRate;
    };
}