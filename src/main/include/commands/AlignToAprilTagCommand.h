#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/VisionSubsystem.h"
#include "subsystems/DriveSubsystem.h"

class AlignToAprilTagCommand : public frc2::CommandHelper<frc2::CommandBase, AlignToAprilTagCommand> {
public:
    AlignToAprilTagCommand(VisionSubsystem* visionSubsystem, DriveSubsystem* driveSubsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    VisionSubsystem* m_visionSubsystem;
    DriveSubsystem* m_driveSubsystem;
};