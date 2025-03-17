#pragma once

#include <units/angle.h>
#include <units/time.h>

namespace CoralConstants {
    // Motor ports
    constexpr int kWristPort = 13;
    constexpr int kIntakePort = 1;

    // Wrist constants
    constexpr units::degree_t kWristStartAngle = 50_deg;
    constexpr units::degree_t kWristDegreeMin = 50_deg;
    constexpr units::degree_t kWristDegreeMax = 290_deg;
    constexpr units::degree_t kWristAngleDeadzone = 2_deg;
    constexpr double kTurnsPerDegree = (20.0 / 1.0) * (3.0 / 1.0) / 360.0; // Adjust based on gear ratio
    constexpr double kMaxFeedForward = 0.35; // Feedforward to counteract gravity

    // PID constants for wrist
    constexpr double kPWrist = 1.0;

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