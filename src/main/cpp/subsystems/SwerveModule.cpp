#include "subsystems/DriveSubsystem/SwerveModule.h"

SwerveModule::SwerveModule(hardware::TalonFX *drivingMotor, rev::CANSparkMax *turningMotor, 
                           DutyCycleEncoder *thetaEncoder, double thetaEncoderOffset)
    : driveMotor(drivingMotor), sparkMaxTurn(turningMotor), neoEncoder(thetaEncoder),
      turnEncoderOffset(thetaEncoderOffset) {
    // Configure the turn motor
    sparkMaxTurn->RestoreFactoryDefaults();
    sparkMaxTurn->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    sparkMaxTurn->SetSmartCurrentLimit(40);
    sparkMaxTurn->SetInverted(false);

    // Configure the PID controller for the turn motor
    neoController.SetP(0.008);
    neoController.SetI(0.0);
    neoController.SetD(0.0);
    neoController.SetTolerance(1.0); // Tolerance in degrees

    // Reset the encoder position
    neoEncoder->SetPositionOffset(turnEncoderOffset);
}

units::meter_t SwerveModule::GetDriveEncoderDistance() const {
    return units::meter_t{driveMotor->GetPosition().GetValue().value() * DriveConstants::kDriveDistancePerRev};
}

units::degree_t SwerveModule::GetTurnEncoderAngle() const {
    double encoderPosition = neoEncoder->GetAbsolutePosition() - turnEncoderOffset;
    return units::degree_t{encoderPosition * 360.0};
}

units::meters_per_second_t SwerveModule::GetDriveEncoderRate() const {
    return units::meters_per_second_t{driveMotor->GetVelocity().GetValue().value() * DriveConstants::kDriveDistancePerRev};
}

frc::SwerveModuleState SwerveModule::GetState() const {
    return frc::SwerveModuleState{GetDriveEncoderRate(), frc::Rotation2d{GetTurnEncoderAngle()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
    return frc::SwerveModulePosition{GetDriveEncoderDistance(), frc::Rotation2d{GetTurnEncoderAngle()}};
}

double SwerveModule::PlaceInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound = scopeReference - 180.0;
    double upperBound = scopeReference + 180.0;
    while (newAngle < lowerBound) {
        newAngle += 360.0;
    }
    while (newAngle > upperBound) {
        newAngle -= 360.0;
    }
    return newAngle;
}

frc::SwerveModuleState SwerveModule::Optimize(const frc::SwerveModuleState& desiredState, frc::Rotation2d currentAngle) {
    double targetAngle = PlaceInAppropriate0To360Scope(currentAngle.Degrees().value(), desiredState.angle.Degrees().value());
    double delta = targetAngle - currentAngle.Degrees().value();
    if (std::abs(delta) > 90.0) {
        return frc::SwerveModuleState{-desiredState.speed, frc::Rotation2d{units::degree_t{targetAngle + 180.0}}};
    }
    return frc::SwerveModuleState{desiredState.speed, frc::Rotation2d{units::degree_t{targetAngle}}};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state) {
    // Optimize the state to minimize rotation
    auto optimizedState = Optimize(state, frc::Rotation2d{GetTurnEncoderAngle()});

    // Set the drive motor speed
    driveMotor->SetControl(ctre::phoenix6::controls::VelocityVoltage{optimizedState.speed.value() / DriveConstants::kDriveDistancePerRev});

    // Set the turn motor angle using PID
    double targetAngle = optimizedState.angle.Degrees().value();
    double currentAngle = GetTurnEncoderAngle().value();
    double output = neoController.Calculate(currentAngle, targetAngle);
    sparkMaxTurn->Set(output);
}

void SwerveModule::RunPID() {
    // Run the PID controller for the turn motor
    double currentAngle = GetTurnEncoderAngle().value();
    double targetAngle = neoController.GetSetpoint();
    double output = neoController.Calculate(currentAngle, targetAngle);
    sparkMaxTurn->Set(output);
}

void SwerveModule::SetDrivePower(double power) {
    driveMotor->Set(power);
}

void SwerveModule::SetTurnPower(double power) {
    sparkMaxTurn->Set(power);
}

void SwerveModule::ResetEncoders() {
    driveMotor->SetPosition(0.0);
    neoEncoder->Reset();
}