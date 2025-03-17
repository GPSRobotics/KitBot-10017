#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/UltrasonicSensor.h"

class IntakeCoralCommand : public frc2::CommandHelper<frc2::CommandBase, IntakeCoralCommand> {
public:
    IntakeCoralCommand(IntakeSubsystem* intakeSubsystem, UltrasonicSensor* ultrasonicSensor);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    IntakeSubsystem* m_intakeSubsystem;
    UltrasonicSensor* m_ultrasonicSensor;
};