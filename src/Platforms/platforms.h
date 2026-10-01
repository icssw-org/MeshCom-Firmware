// All platforms, as one header

#ifndef __PLATFORMS_H__
#define __PLATFORMS_H__

#include <Arduino.h>

// Supported
#include "Platforms/WirelessPaper/WirelessPaper.h"
#include "Platforms/VisionMasterE213/VisionMasterE213.h"
#include "Platforms/VisionMasterE290/VisionMasterE290.h"

#include "Platforms/ESP32/ESP32.h"

// Fallback
#include "Platforms/fallback/fallback.h"

#endif