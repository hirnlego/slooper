#pragma once

#include "wreath/stereo_looper.h"

namespace slooper
{
    using namespace wreath;

    constexpr float kMinSpeedMult{0.02f};
    constexpr float kMaxSpeedMult{4.f};

    StereoLooper looper;
}