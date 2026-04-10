#pragma once

#include "common/types.h"
#include "common/config.h"

namespace truggy {

// Thread 3: Actuation loop. Reads IMU/encoder telemetry from Arduino,
// writes control commands. Runs at 100 Hz.
void actuation_loop(shared_bus_t* bus, const config_t& cfg);

} // namespace truggy
