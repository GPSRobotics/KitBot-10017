#include "subsystems/CoralSubsystem/CoralSubsystem.h"

CoralSubsystem::CoralSubsystem()
    : wristMotor{kWristPort, CANSparkMax::MotorType::kBrushless},
      intakeMotor{kIntakePort, CANSparkMax::MotorType::kBrushless},
      wristEncoder{wristMotor.GetEncoder()},
      wristPID{wristMotor.GetPIDController()} {
    // Configure motors and encoders
    ConfigureWrist();
    ConfigureIntake();

    // Initialize SmartDashboard
    frc::SmartDashboard::PutNumber("Coral Wrist Target Angle", wristAngle.value());
    frc::SmartDashboard::PutNumber("Coral Wrist Power", wristPower);
    frc::SmartDashboard::PutNumber("Coral Intake Power", intakePower);
}

void CoralSubsystem::Periodic() {
    // Update wrist control
    if (wristState == WristStates::kWristOff) {
        wristMotor.Set(0.0);
    } else if (wristState == WristStates::kWristPowerMode) {
        wristMotor.Set(wristPower);
    } else if (wristState == WristStates::kWristAngleMode) {
        // Calculate feedforward to counteract gravity
        double feedForward = CalculateFeedForward(wristAngle);
        units::turn_t posTarget{(wristAngle - kWristStartAngle) / kTurnsPerDegree};

        // Set target position with feedforward
        wristPID.SetReference(posTarget.value(), CANSparkMax::ControlType::kPosition, 0, feedForward);

        // Log data to SmartDashboard
        frc::SmartDashboard::PutNumber("Coral Wrist Position", wristEncoder.GetPosition());
        frc::SmartDashboard::PutNumber("Coral Wrist Angle", GetAngle().value());
        frc::SmartDashboard::PutNumber("Coral Wrist Feedforward", feedForward);
    }

    // Update intake control
    if (intakeState == IntakeStates::kIntakeOff) {
        intakeMotor.Set(0.0);
    } else {
        intakeMotor.Set(intakePower);
    }

    // Log intake state to SmartDashboard
    frc::SmartDashboard::PutString("Coral Intake State", intakeState == IntakeStates::kIntakeOff ? "Off" : "On");
}

void CoralSubsystem::SetIntakePower(double power) {
    intakePower = std::clamp(power, -1.0, 1.0);
}

double CoralSubsystem::GetIntakePower() const {
    return intakePower;
}

void CoralSubsystem::SetIntakeState(IntakeStates state) {
    intakeState = state;
}

IntakeStates CoralSubsystem::GetIntakeState() const {
    return intakeState;
}

void CoralSubsystem::SetWristPower(double power) {
    wristPower = std::clamp(power, -1.0, 1.0);
}

double CoralSubsystem::GetWristPower() const {
    return wristPower;
}

void CoralSubsystem::SetWristState(WristStates state) {
    wristState = state;
}

WristStates CoralSubsystem::GetWristState() const {
    return wristState;
}

void CoralSubsystem::SetTargetAngle(units::degree_t angle) {
    wristAngle = std::clamp(angle, kWristDegreeMin, kWristDegreeMax);
    frc::SmartDashboard::PutNumber("Coral Wrist Target Angle", wristAngle.value());
}

units::degree_t CoralSubsystem::GetAngle() const {
    return units::degree_t{wristEncoder.GetPosition() / kTurnsPerDegree} + kWristStartAngle;
}

bool CoralSubsystem::IsAtTarget() const {
    auto angle = GetAngle();
    return angle > wristAngle - (kWristAngleDeadzone / 2) && angle < wristAngle + (kWristAngleDeadzone / 2);
}

frc2::CommandPtr CoralSubsystem::GetMoveCommand(units::degree_t target) {
    return frc2::cmd::RunOnce([this, target]() { SetTargetAngle(target); }, {this});
}

void CoralSubsystem::ConfigureWrist() {
    wristMotor.RestoreFactoryDefaults();
    wristMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    wristMotor.SetSmartCurrentLimit(40);
    wristMotor.SetInverted(true);

    wristPID.SetP(kPWrist);
    wristPID.SetI(0.0);
    wristPID.SetD(0.0);
    wristPID.SetOutputRange(-1.0, 1.0);

    wristEncoder.SetPosition(0.0);
}

void CoralSubsystem::ConfigureIntake() {
    intakeMotor.RestoreFactoryDefaults();
    intakeMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    intakeMotor.SetSmartCurrentLimit(40);
    intakeMotor.SetInverted(false);
}

double CoralSubsystem::CalculateFeedForward(units::degree_t angle) const {
    return std::sin(angle.value() * (M_PI / 180.0)) * kMaxFeedForward;
}