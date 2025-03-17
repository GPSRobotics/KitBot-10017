#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h> // For Spark MAX motors
#include <units/angle.h>
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

using namespace frc;
using namespace rev;
using namespace AlgaeConstants;

class AlgaeSubsystem : public frc2::SubsystemBase {
public:
    AlgaeSubsystem();

    void Periodic() override;

    // Intake control
    void SetIntakePower(double power);
    double GetIntakePower() const;
    void SetIntakeState(IntakeStates state);
    IntakeStates GetIntakeState() const;

    // Wrist control
    void SetWristPower(double power);
    double GetWristPower() const;
    void SetWristState(WristStates state);
    WristStates GetWristState() const;
    void SetTargetAngle(units::degree_t angle);
    units::degree_t GetAngle() const;
    bool IsAtTarget() const;

    // Command factory
    frc2::CommandPtr GetMoveCommand(units::degree_t target);

private:
    // Motors
    CANSparkMax wristMotor;
    CANSparkMax intakeMotor;

    // Encoder for wrist position
    SparkMaxRelativeEncoder wristEncoder;

    // PID controller for wrist
    SparkMaxPIDController wristPID;

    // Subsystem states
    IntakeStates intakeState = IntakeStates::kIntakeOff;
    WristStates wristState = WristStates::kWristOff;

    // Power and target angle
    double intakePower = 0.0;
    double wristPower = 0.0;
    units::degree_t wristAngle = kWristStartAngle;

    // Configuration methods
    void ConfigureWrist();
    void ConfigureIntake();

    // Helper methods
    double CalculateFeedForward(units::degree_t angle) const;
};