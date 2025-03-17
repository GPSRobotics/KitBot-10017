#pragma once

#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/DutyCycleEncoder.h>
#include <rev/CANSparkMax.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;
using namespace rev::spark;

class SwerveModule {
public:
    SwerveModule(hardware::TalonFX *drivingMotor, SparkMax *turningMotor, 
                 DutyCycleEncoder *thetaEncoder, double thetaEncoderOffset = 0);

    units::meter_t GetDriveEncoderDistance() const;
    units::degree_t GetTurnEncoderAngle() const;
    units::meters_per_second_t GetDriveEncoderRate() const;
    frc::SwerveModuleState GetState() const;
    frc::SwerveModulePosition GetPosition() const;
    static double PlaceInAppropriate0To360Scope(double scopeReference, double newAngle);
    static frc::SwerveModuleState Optimize(const frc::SwerveModuleState& desiredState, frc::Rotation2d currentAngle);
    void SetDesiredState(const frc::SwerveModuleState& state);
    void RunPID();
    void SetDrivePower(double power);
    void SetTurnPower(double power);
    void ResetEncoders();

private:
    hardware::TalonFX *driveMotor;
    CANSparkMax *sparkMaxTurn;
    DutyCycleEncoder *neoEncoder;
    double turnEncoderOffset;
    frc::PIDController neoController{0.008, 0.0, 0.0};
};