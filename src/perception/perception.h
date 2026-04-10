#pragma once

#include "common/types.h"
#include "common/config.h"

namespace truggy {

// Thread 0: Perception loop
// ZED capture (720p/30fps) + VIO pose + segmentation + costmap generation
void perception_loop(shared_bus_t* bus, const config_t& cfg);

} // namespace truggy
