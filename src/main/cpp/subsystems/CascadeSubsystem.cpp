#include "subsystems/CascadeSubsystem/CascadeSubsystem.h"

CascadeSubsystem::CascadeSubsystem()
    : motor{kMotorPort, CANSparkMax::MotorType::kBrushless},
      encoder{motor.GetEncoder()},
      pidController{motor.GetPIDController()} {
    // Configure motors and encoders
    ConfigureMotors();

    // Initialize SmartDashboard
    frc::SmartDashboard::PutNumber("Cascade Target Position", targetPosition.value());
    frc::SmartDashboard::PutNumber("Cascade Power", power);
}

void CascadeSubsystem::Periodic() {
    // Update target position from SmartDashboard (for testing)
    targetPosition = units::meter_t{frc::SmartDashboard::GetNumber("Cascade Target Position", targetPosition.value())};

    // Control logic based on state
    if (state == CascadeStates::kOff) {
        motor.Set(0.0);
    } else if (state == CascadeStates::kPowerMode) {
        motor.Set(power);
    } else if (state == CascadeStates::kPositionMode) {
        // Convert position to turns and set PID reference
        units::turn_t turnsTarget = PositionToTurns(targetPosition);
        pidController.SetReference(turnsTarget.value(), CANSparkMax::ControlType::kPosition);

        // Log data to SmartDashboard
        frc::SmartDashboard::PutNumber("Cascade Actual Position", GetPosition().value());
        frc::SmartDashboard::PutNumber("Cascade Target Turns", turnsTarget.value());
    }
}

void CascadeSubsystem::SetState(CascadeStates newState) {
    state = newState;
}

CascadeStates CascadeSubsystem::GetState() const {
    return state;
}

void CascadeSubsystem::SetPower(double newPower) {
    power = std::clamp(newPower, -1.0, 1.0);
    frc::SmartDashboard::PutNumber("Cascade Power", power);
}

double CascadeSubsystem::GetPower() const {
    return power;
}

void CascadeSubsystem::SetTargetPosition(units::meter_t newPosition) {
    targetPosition = std::clamp(newPosition, kCascadeMeterMin, kCascadeMeterMax);
    frc::SmartDashboard::PutNumber("Cascade Target Position", targetPosition.value());
}

units::meter_t CascadeSubsystem::GetPosition() const {
    return units::meter_t{encoder.GetPosition() / kTurnsPerMeter} + kStartPosition;
}

bool CascadeSubsystem::IsAtTarget() const {
    auto currentPosition = GetPosition();
    return currentPosition > targetPosition - (kPositionDeadzone / 2) &&
           currentPosition < targetPosition + (kPositionDeadzone / 2);
}

void CascadeSubsystem::SetBrakeMode(bool brake) {
    motor.SetIdleMode(brake ? CANSparkMax::IdleMode::kBrake : CANSparkMax::IdleMode::kCoast);
}

frc2::CommandPtr CascadeSubsystem::GetMoveCommand(units::meter_t target) {
    return frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this, target]() { SetTargetPosition(target); }, {this}),
        frc2::cmd::WaitUntil([this]() { return IsAtTarget(); })
    );
}

void CascadeSubsystem::ConfigureMotors() {
    motor.RestoreFactoryDefaults();
    motor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    motor.SetSmartCurrentLimit(40);
    motor.SetInverted(false);

    pidController.SetP(kP);
    pidController.SetI(0.0);
    pidController.SetD(0.0);
    pidController.SetOutputRange(-1.0, 1.0);

    encoder.SetPosition(0.0);
}

units::turn_t CascadeSubsystem::PositionToTurns(units::meter_t position) const {
    return units::turn_t{(position - kStartPosition).value() * kTurnsPerMeter};
}