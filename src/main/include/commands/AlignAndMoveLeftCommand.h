#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/VisionSubsystem.h"
#include "subsystems/DriveSubsystem.h"

class AlignAndMoveLeftCommand : public frc2::CommandHelper<frc2::CommandBase, AlignAndMoveLeftCommand> {
public:
    AlignAndMoveLeftCommand(VisionSubsystem* visionSubsystem, DriveSubsystem* driveSubsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    VisionSubsystem* m_visionSubsystem;
    DriveSubsystem* m_driveSubsystem;
    bool m_isAligned = false;
};