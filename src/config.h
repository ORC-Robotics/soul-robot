#pragma once

namespace config
{
constexpr int kDashboardPort = 8080;
constexpr int kTelemetryPeriodMs = 500;
constexpr int kCommandTimeoutMs = 2000;

// Raspberry Pi BCM GPIO numbers for an L298N bridge.
// Adjust these to match the robot wiring.
constexpr int kLeftEnablePin = 12;
constexpr int kLeftInput1Pin = 5;
constexpr int kLeftInput2Pin = 6;

constexpr int kRightEnablePin = 13;
constexpr int kRightInput1Pin = 20;
constexpr int kRightInput2Pin = 21;

// This first implementation uses digital motor enable. A value with absolute
// magnitude below this threshold is treated as stopped.
constexpr double kMotorDeadband = 0.05;
}
