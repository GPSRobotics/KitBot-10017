#pragma once

#include <units/angle.h>
#include <units/time.h>

namespace AlgaeConstants {
    // Motor ports
    constexpr int kWristPort = 8;
    constexpr int kIntakePort = 9;

    // Wrist constants
    constexpr units::degree_t kWristStartAngle = 120_deg;
    constexpr units::degree_t kWristDegreeMin = -45_deg;
    constexpr units::degree_t kWristDegreeMax = 110_deg;
    constexpr units::degree_t kWristAngleDeadzone = 2_deg;
    constexpr double kTurnsPerDegree = 1.0 / 360.0; // Adjust based on gear ratio
    constexpr double kMaxFeedForward = 0.35; // Feedforward to counteract gravity

    // PID constants for wrist
    constexpr double kWristP = 0.1;
    constexpr double kWristI = 0.0;
    constexpr double kWristD = 0.0;

    // Intake constants
    constexpr double kIntakeDefaultPower = 0.8;

    // Subsystem states
    enum IntakeStates {
        kIntakeOff,
        kIntakePowerMode
    };

    enum WristStates {
        kWristOff,
        kWristPowerMode,
        kWristAngleMode
    };
}