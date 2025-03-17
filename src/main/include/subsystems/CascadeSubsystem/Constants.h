#pragma once

#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>

namespace CascadeConstants {
    // Motor and encoder ports
    constexpr int kMotorPort = 12;
    constexpr int kEncoderPort = 14;

    // Subsystem states
    enum CascadeStates {
        kOff,
        kPowerMode,
        kPositionMode
    };

    // Default power in power mode
    constexpr double kDefaultPower = 0.0;

    // Position constants
    constexpr units::meter_t kStartPosition = 0.92_m;
    constexpr units::meter_t kCascadeMeterMin = kStartPosition;
    constexpr units::meter_t kCascadeMeterMax = 1.45_m;
    constexpr units::meter_t kPositionDeadzone = 3.0_cm;

    // Conversion factor
    constexpr double kTurnsPerMeter = 329.30312383742559;

    // PID constants
    constexpr double kP = 0.05;
    constexpr double kI = 0.0;
    constexpr double kD = 0.0;
}