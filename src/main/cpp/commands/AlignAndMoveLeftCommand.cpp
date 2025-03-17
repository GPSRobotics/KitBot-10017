#include "commands/AlignAndMoveLeftCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

AlignAndMoveLeftCommand::AlignAndMoveLeftCommand(VisionSubsystem* visionSubsystem, DriveSubsystem* driveSubsystem)
    : m_visionSubsystem(visionSubsystem), m_driveSubsystem(driveSubsystem) {
    AddRequirements({m_visionSubsystem, m_driveSubsystem});
}

void AlignAndMoveLeftCommand::Initialize() {
    // Align with the AprilTag
    m_driveSubsystem->ResetAlignment();
}

void AlignAndMoveLeftCommand::Execute() {
    if (!m_isAligned) {
        // Align with the AprilTag
        double yawOffset = m_visionSubsystem->GetTargetYaw();
        m_driveSubsystem->AlignToTarget(yawOffset);

        // Check if aligned
        if (std::abs(yawOffset) < 1.0) {
            m_isAligned = true;
            m_driveSubsystem->ResetEncoders(); // Reset encoders for the move left phase
        }
    } else {
        // Move left a certain distance (e.g., 1 meter)
        m_driveSubsystem->Drive(0.0_mps, -1.0_mps, 0.0_rad_per_s, false);
    }
}

bool AlignAndMoveLeftCommand::IsFinished() {
    // Finish when the robot has moved left the desired distance
    return m_isAligned && m_driveSubsystem->GetDistanceDriven() >= 1.0_m;
}

void AlignAndMoveLeftCommand::End(bool interrupted) {
    // Stop the robot
    m_driveSubsystem->Stop();
}