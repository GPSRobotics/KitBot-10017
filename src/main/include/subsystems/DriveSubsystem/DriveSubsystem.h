#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DutyCycleEncoder.h>
#include <rev/SparkMax.h>
#include <rev/SparkMaxAlternateEncoder.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/filter/SlewRateLimiter.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/DriverStation.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

#include "GlobalConstants.h"
#include "SwerveModule.h"

using namespace frc;
using namespace ctre::phoenix6;
using namespace rev::spark;
using namespace DriveConstants;

class DriveSubsystem : public frc2::SubsystemBase {
public:
    DriveSubsystem(int *targetRef);

    void Periodic() override;

    // Subsystem methods
    void HandleTargeting();
    void Drive(ChassisSpeeds speeds, bool applyLimits = false, bool fieldRelative = false);
    void ResetEncoders();
    void SetInverted(bool inverted);
    void SetModuleStates(wpi::array<SwerveModuleState, 4> desiredStates, bool desaturate = true);
    wpi::array<SwerveModuleState, 4> GetModuleStates() const;
    frc2::CommandPtr FollowPathCommand(std::string path);
    void SetDrivePower(double power);
    void SetTurnPower(double power);
    units::degree_t GetAngle();
    void ZeroHeading();
    double GetTurnRate();
    Rotation2d GetRotation();
    Pose2d GetPose();
    void ResetOdometry(Pose2d pose);
    void ResetRateLimiter();
    void SetLimiting(bool state);
    void SetBrakeMode(bool state);
    void ConfigDriveMotors();
    void ConfigAutonController();
    double GetPitch();
    void SetPoseToHold(Pose2d target);
    Pose2d GetPoseToHold();
    void ResetFromJetson();
    void SetThetaToHold(Rotation2d target);
    bool GetOmegaOverride();
    void SetOmegaOverride(bool state);
    bool GetYOverride();
    void SetYOverride(bool state);
    bool IsAtTarget();
    units::meter_t GetDistToTarget();

private:
    // Motor controllers
    hardware::TalonFX backLeft;
    hardware::TalonFX frontLeft;
    hardware::TalonFX backRight;
    hardware::TalonFX frontRight;

    // Turn motors
    SparkMax backLeftTheta;
    SparkMax frontLeftTheta;
    SparkMax backRightTheta;
    SparkMax frontRightTheta;

    // Encoders
    DutyCycleEncoder blEncoder;
    DutyCycleEncoder flEncoder;
    DutyCycleEncoder brEncoder;
    DutyCycleEncoder frEncoder;

    // Swerve modules
    SwerveModule s_backLeft;
    SwerveModule s_frontLeft;
    SwerveModule s_backRight;
    SwerveModule s_frontRight;

    // Gyro
    hardware::Pigeon2 gyro;

    // Odometry
    SwerveDriveOdometry<4> odometry;

    // Rate limiters
    SlewRateLimiter<units::meters_per_second> xAccel;
    SlewRateLimiter<units::meters_per_second> yAccel;
    SlewRateLimiter<units::meters_per_second> xDecel;
    SlewRateLimiter<units::meters_per_second> yDecel;

    // State variables
    bool enableLimiting = false;
    bool omegaOverride = false;
    bool yOverride = false;
    bool targetUsingLimelight = true;
    bool isAtTarget = false;
    Rotation2d targetTheta{0.0_deg};
    Pose2d poseToHold{};
    PIDController thetaHoldController{0.13, 0.0, 0.0};
    int *thetaTarget;
    double lastX = 0.0;
    double lastY = 0.0;
    units::meter_t distFromTarget{0.0_m};
    Field2d fieldWidget;
};