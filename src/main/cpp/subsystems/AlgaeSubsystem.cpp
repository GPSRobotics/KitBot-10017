#include "subsystems/AlgaeSubsystem/AlgaeSubsystem.h"

AlgaeSubsystem::AlgaeSubsystem()
    : wristMotor{kWristPort, CANSparkMax::MotorType::kBrushless},
      intakeMotor{kIntakePort, CANSparkMax::MotorType::kBrushless},
      wristEncoder{wristMotor.GetEncoder()},
      wristPID{wristMotor.GetPIDController()} {
    // Configure motors and encoders
    ConfigureWrist();
    ConfigureIntake();

    // Initialize SmartDashboard
    frc::SmartDashboard::PutNumber("Algae Wrist Target Angle", wristAngle.value());
    frc::SmartDashboard::PutNumber("Algae Wrist Power", wristPower);
    frc::SmartDashboard::PutNumber("Algae Intake Power", intakePower);
}

void AlgaeSubsystem::Periodic() {
    // Wrist control
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
        frc::SmartDashboard::PutNumber("Algae Wrist Position", wristEncoder.GetPosition());
        frc::SmartDashboard::PutNumber("Algae Wrist Angle", GetAngle().value());
        frc::SmartDashboard::PutNumber("Algae Wrist Feedforward", feedForward);
    }

    // Intake control
    if (intakeState == IntakeStates::kIntakeOff) {
        intakeMotor.Set(0.0);
    } else {
        intakeMotor.Set(intakePower);
    }

    // Log intake state to SmartDashboard
    frc::SmartDashboard::PutString("Algae Intake State", intakeState == IntakeStates::kIntakeOff ? "Off" : "On");
}

void AlgaeSubsystem::SetIntakePower(double power) {
    intakePower = std::clamp(power, -1.0, 1.0);
}

double AlgaeSubsystem::GetIntakePower() const {
    return intakePower;
}

void AlgaeSubsystem::SetIntakeState(IntakeStates state) {
    intakeState = state;
}

IntakeStates AlgaeSubsystem::GetIntakeState() const {
    return intakeState;
}

void AlgaeSubsystem::SetWristPower(double power) {
    wristPower = std::clamp(power, -1.0, 1.0);
}

double AlgaeSubsystem::GetWristPower() const {
    return wristPower;
}

void AlgaeSubsystem::SetWristState(WristStates state) {
    wristState = state;
}

WristStates AlgaeSubsystem::GetWristState() const {
    return wristState;
}

void AlgaeSubsystem::SetTargetAngle(units::degree_t angle) {
    wristAngle = std::clamp(angle, kWristDegreeMin, kWristDegreeMax);
    frc::SmartDashboard::PutNumber("Algae Wrist Target Angle", wristAngle.value());
}

units::degree_t AlgaeSubsystem::GetAngle() const {
    return units::degree_t{wristEncoder.GetPosition() / kTurnsPerDegree} + kWristStartAngle;
}

bool AlgaeSubsystem::IsAtTarget() const {
    auto angle = GetAngle();
    return angle > wristAngle - (kWristAngleDeadzone / 2) && angle < wristAngle + (kWristAngleDeadzone / 2);
}

frc2::CommandPtr AlgaeSubsystem::GetMoveCommand(units::degree_t target) {
    return frc2::cmd::RunOnce([this, target]() { SetTargetAngle(target); }, {this});
}

void AlgaeSubsystem::ConfigureWrist() {
    wristMotor.RestoreFactoryDefaults();
    wristMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    wristMotor.SetSmartCurrentLimit(40);
    wristMotor.SetInverted(false);

    wristPID.SetP(kWristP);
    wristPID.SetI(kWristI);
    wristPID.SetD(kWristD);
    wristPID.SetOutputRange(-1.0, 1.0);

    wristEncoder.SetPosition(0.0);
}

void AlgaeSubsystem::ConfigureIntake() {
    intakeMotor.RestoreFactoryDefaults();
    intakeMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    intakeMotor.SetSmartCurrentLimit(40);
    intakeMotor.SetInverted(false);
}

double AlgaeSubsystem::CalculateFeedForward(units::degree_t angle) const {
    return std::sin(angle.value() * (M_PI / 180.0)) * kMaxFeedForward;
}