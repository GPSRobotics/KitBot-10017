#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/LiftSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

class PlaceCoralTier4Command : public frc2::CommandHelper<frc2::CommandBase, PlaceCoralTier4Command> {
public:
    PlaceCoralTier4Command(LiftSubsystem* liftSubsystem, IntakeSubsystem* intakeSubsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    LiftSubsystem* m_liftSubsystem;
    IntakeSubsystem* m_intakeSubsystem;
};