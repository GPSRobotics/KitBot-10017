#pragma once

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <numbers>
#include <frc/geometry/Pose2d.h>

namespace DriveConstants {
    // Motor ports
    constexpr int kBackLeftPort = 0;
    constexpr int kFrontLeftPort = 1;
    constexpr int kBackRightPort = 2;
    constexpr int kFrontRightPort = 3;

    // Turn motor ports
    constexpr int kBackLeftThetaPort = 4;
    constexpr int kFrontLeftThetaPort = 5;
    constexpr int kBackRightThetaPort = 6;
    constexpr int kFrontRightThetaPort = 7;

    // Encoder ports
    constexpr int kBackLeftEncoderPort = 0;
    constexpr int kFrontLeftEncoderPort = 1;
    constexpr int kBackRightEncoderPort = 2;
    constexpr int kFrontRightEncoderPort = 3;

    // Encoder offsets (in rotations)
    constexpr double kBLeftMagPos = 0.220534655513366;
    constexpr double kFLeftMagPos = 0.24972238124306;
    constexpr double kBRightMagPos = 0.020010375500259;
    constexpr double kFRightMagPos = 0.997154824928871 - 0.50;

    // Physical dimensions
    constexpr units::meter_t kWheelRadius = 0.0381_m;
    constexpr units::meter_t kDriveBaseRadius = 0.319786_m;
    constexpr double kDriveRatio = 1 / 4.13; // Gear ratio for drive motors
    constexpr double kTurnRatio = 1 / 10.29; // Gear ratio for turn motors
    constexpr units::meter_t kDriveDistancePerRev = (2 * std::numbers::pi * kWheelRadius) * kDriveRatio;

    // PID constants
    constexpr double kDriveP = 0.08; // Proportional gain for drive motors
    constexpr double kDriveV = 0.1345313787460327; // Velocity feedforward for drive motors
    constexpr double kTurnP = 80.0; // Proportional gain for turn motors
    constexpr double kTurnPRatio = 10.279000282287598; // Scaling factor for turn motor PID

    // Rate limiting
    constexpr units::meters_per_second_squared_t kDriveAccelerationLimit = 9.0_mps_sq;
    constexpr units::meters_per_second_squared_t kDriveDecelerationLimit = 7.0_mps_sq;
    constexpr units::meters_per_second_t kDriveTranslationLimit = 3.3_mps;

    // Deadzones
    constexpr double kDriveDeadzone = 0.2; // Deadzone for drive joystick
    constexpr double kTurnDeadzone = 0.1; // Deadzone for turn joystick
    constexpr double kThetaDeadzone = 2.0; // Deadzone for theta control (degrees)

    // Alignment constants
    constexpr double kTxAdjust = 0.5; // Adjust for target alignment
    constexpr double kAlignP = 0.5; // Proportional gain for alignment
    constexpr double kPVelTurnOffset = -2.0; // Velocity offset for turning
    constexpr double kPVelDistOffset = 0.0; // Velocity offset for distance
    constexpr double kDistMultiplier = 1.5; // Multiplier for distance calculations
    constexpr int kDistSamples = 5; // Number of samples for distance averaging

    // Robot properties
    constexpr units::kilogram_t kRobotWeight = 54.43_kg; // Weight of the robot
    constexpr double kMOI = 60; // Moment of inertia (placeholder value)

    // Swerve module locations relative to the center of the robot
    constexpr frc::Translation2d frontLeftLocation{0.319786_m, 0.319786_m};
    constexpr frc::Translation2d frontRightLocation{0.319786_m, -0.319786_m};
    constexpr frc::Translation2d backLeftLocation{-0.319786_m, 0.319786_m};
    constexpr frc::Translation2d backRightLocation{-0.319786_m, -0.319786_m};

    // Kinematics for swerve drive
    constexpr frc::SwerveDriveKinematics<4> kDriveKinematics{
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation};
}