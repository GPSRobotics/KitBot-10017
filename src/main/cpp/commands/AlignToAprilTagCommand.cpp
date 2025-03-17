#include "commands/AlignToAprilTagCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

AlignToAprilTagCommand::AlignToAprilTagCommand(VisionSubsystem* visionSubsystem, DriveSubsystem* driveSubsystem)
    : m_visionSubsystem(visionSubsystem), m_driveSubsystem(driveSubsystem) {
    AddRequirements({m_visionSubsystem, m_driveSubsystem});
}

void AlignToAprilTagCommand::Initialize() {
    // Reset any previous alignment state
    m_driveSubsystem->ResetAlignment();
}

void AlignToAprilTagCommand::Execute() {
    // Get the yaw offset from the vision subsystem
    double yawOffset = m_visionSubsystem->GetTargetYaw();

    // Drive the robot to align with the AprilTag
    m_driveSubsystem->AlignToTarget(yawOffset);
}

bool AlignToAprilTagCommand::IsFinished() {
    // Finish when the robot is aligned (yaw offset is within tolerance)
    return std::abs(m_visionSubsystem->GetTargetYaw()) < 1.0; // Tolerance of 1 degree
}

void AlignToAprilTagCommand::End(bool interrupted) {
    // Stop the robot
    m_driveSubsystem->Stop();
}