#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h> // For Spark MAX motors
#include <units/length.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

using namespace frc;
using namespace rev;
using namespace CascadeConstants;

class CascadeSubsystem : public frc2::SubsystemBase {
public:
    CascadeSubsystem();

    void Periodic() override;

    // State control
    void SetState(CascadeStates state);
    CascadeStates GetState() const;

    // Power control
    void SetPower(double power);
    double GetPower() const;

    // Position control
    void SetTargetPosition(units::meter_t position);
    units::meter_t GetPosition() const;
    bool IsAtTarget() const;

    // Brake mode
    void SetBrakeMode(bool brake);

    // Command factory
    frc2::CommandPtr GetMoveCommand(units::meter_t target);

private:
    // Motor and encoder
    CANSparkMax motor;
    SparkMaxRelativeEncoder encoder;
    SparkMaxPIDController pidController;

    // Subsystem state
    CascadeStates state = CascadeStates::kPositionMode;
    double power = kDefaultPower;
    units::meter_t targetPosition = kStartPosition;

    // Configuration methods
    void ConfigureMotors();

    // Helper methods
    units::turn_t PositionToTurns(units::meter_t position) const;
};