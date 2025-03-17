#include "subsystems/DriveSubsystem/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem(int *targetRef)
    : backLeft{kBackLeftPort}, frontLeft{kFrontLeftPort}, backRight{kBackRightPort}, frontRight{kFrontRightPort},
      backLeftTheta{kBackLeftThetaPort, SparkLowLevel::MotorType::kBrushed},
      frontLeftTheta{kFrontLeftThetaPort, SparkLowLevel::MotorType::kBrushed},
      backRightTheta{kBackRightThetaPort, SparkLowLevel::MotorType::kBrushed},
      frontRightTheta{kFrontRightThetaPort, SparkLowLevel::MotorType::kBrushed},
      blEncoder{kBackLeftEncoderPort}, flEncoder{kFrontLeftEncoderPort},
      brEncoder{kBackRightEncoderPort}, frEncoder{kFrontRightEncoderPort},
      s_backLeft{&backLeft, &backLeftTheta, &blEncoder, kBLeftMagPos},
      s_frontLeft{&frontLeft, &frontLeftTheta, &flEncoder, kFLeftMagPos},
      s_backRight{&backRight, &backRightTheta, &brEncoder, kBRightMagPos},
      s_frontRight{&frontRight, &frontRightTheta, &frEncoder, kFRightMagPos},
      gyro{0},
      odometry{kDriveKinematics, GetRotation(), {s_frontLeft.GetPosition(), s_frontRight.GetPosition(),
                s_backLeft.GetPosition(), s_backRight.GetPosition()}, Pose2d{0.0_m, 0.0_m, 0.0_deg}},
      xAccel{kDriveAccelerationLimit}, yAccel{kDriveAccelerationLimit},
      xDecel{kDriveDecelerationLimit}, yDecel{kDriveDecelerationLimit} {
    thetaTarget = targetRef;
    ConfigDriveMotors();
    ConfigAutonController();
    SmartDashboard::PutBoolean("Limelight Targeting", targetUsingLimelight);
}

void DriveSubsystem::Periodic() {
    // Update odometry
    odometry.Update(GetRotation(), {s_frontLeft.GetPosition(), s_frontRight.GetPosition(),
                                   s_backLeft.GetPosition(), s_backRight.GetPosition()});

    // Update SmartDashboard
    fieldWidget.SetRobotPose(odometry.GetPose());
    auto pose = odometry.GetPose();
    SmartDashboard::PutNumber("poseX", pose.X().value());
    SmartDashboard::PutNumber("poseY", pose.Y().value());
    SmartDashboard::PutNumber("poseAngle", pose.Rotation().Degrees().value());
    SmartDashboard::PutBoolean("isAtTarget", isAtTarget);
    SmartDashboard::PutData("Field", &fieldWidget);

    HandleTargeting();
}

void DriveSubsystem::Drive(ChassisSpeeds speeds, bool applyLimits, bool fieldRelative) {
    units::meters_per_second_t x = speeds.vx;
    units::meters_per_second_t y = speeds.vy;
    units::radians_per_second_t rot = speeds.omega;

    if (omegaOverride) {
        double angle = GetPose().Rotation().Degrees().value();
        double target = SwerveModule::PlaceInAppropriate0To360Scope(thetaHoldController.GetSetpoint(), angle);
        rot = units::radians_per_second_t{thetaHoldController.Calculate(target)};
    }

    if (enableLimiting && applyLimits) {
        x = xAccel.Calculate(x);
        y = yAccel.Calculate(y);
    }

    auto states = kDriveKinematics.ToSwerveModuleStates(
        fieldRelative ? ChassisSpeeds::FromFieldRelativeSpeeds(x, y, rot, GetPose().Rotation())
                     : ChassisSpeeds{x, y, rot});

    if (!applyLimits) {
        kDriveKinematics.DesaturateWheelSpeeds(&states, kDriveTranslationLimit);
    }

    SetModuleStates(states, applyLimits);
}

void DriveSubsystem::SetModuleStates(wpi::array<SwerveModuleState, 4> desiredStates, bool desaturate) {
    if (desaturate) {
        kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, kDriveTranslationLimit);
    }
    s_frontLeft.SetDesiredState(desiredStates[0]);
    s_frontRight.SetDesiredState(desiredStates[1]);
    s_backLeft.SetDesiredState(desiredStates[2]);
    s_backRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ConfigDriveMotors() {
    configs::TalonFXConfiguration driveConfig{};
    driveConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    driveConfig.Slot0.kP = kDriveP;
    driveConfig.Slot0.kV = kDriveV;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kDriveRamp;
    driveConfig.CurrentLimits.SupplyCurrentLimit = kDriveCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.Audio.AllowMusicDurDisable = true;

    backLeft.GetConfigurator().Apply(driveConfig);
    frontLeft.GetConfigurator().Apply(driveConfig);
    backRight.GetConfigurator().Apply(driveConfig);
    frontRight.GetConfigurator().Apply(driveConfig);
}

void DriveSubsystem::ConfigAutonController() {
    AutoBuilder::configureHolonomic(
        [this]() { return GetPose(); },
        [this](Pose2d pose) { ResetOdometry(pose); },
        [this]() { return kDriveKinematics.ToChassisSpeeds(GetModuleStates()); },
        [this](ChassisSpeeds speeds) { Drive(speeds); },
        RobotConfig::fromGUISettings(),
        []() { return DriverStation::GetAlliance() == DriverStation::Alliance::kRed; },
        this
    );
}