#ifndef DEVICE_CONFIG_H_
#define DEVICE_CONFIG_H_

#include <stdint.h>

// Copy this file to device_config.h and update the values for your boat/device.

constexpr const char* kDeviceHostname = "sk-flush";
constexpr const char* kBoatWifiSsid = "your-ssid";
constexpr const char* kBoatWifiPassword = "your-password";
constexpr const char* kSignalKServerHost = "signalk-server.local";
constexpr uint16_t kSignalKServerPort = 3000;

// Time zone configuration for the boat
constexpr const char* kShipTimeZoneName = "US Eastern";
constexpr const char* kShipTimeZonePosix = "EST5EDT,M3.2.0/2,M11.1.0/2";

#endif  // DEVICE_CONFIG_H_
