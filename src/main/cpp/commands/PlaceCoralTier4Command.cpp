#include "commands/PlaceCoralTier4Command.h"
#include <frc/smartdashboard/SmartDashboard.h>

PlaceCoralTier4Command::PlaceCoralTier4Command(LiftSubsystem* liftSubsystem, IntakeSubsystem* intakeSubsystem)
    : m_liftSubsystem(liftSubsystem), m_intakeSubsystem(intakeSubsystem) {
    AddRequirements({m_liftSubsystem, m_intakeSubsystem});
}

void PlaceCoralTier4Command::Initialize() {
    // Raise the lift to tier 4 height (e.g., 2 meters)
    m_liftSubsystem->SetTargetHeight(2.0_m);
}

void PlaceCoralTier4Command::Execute() {
    // Check if the lift has reached the target height
    if (m_liftSubsystem->IsAtTargetHeight()) {
        // Start the intake to place the coral
        m_intakeSubsystem->StartIntake();
    }
}

bool PlaceCoralTier4Command::IsFinished() {
    // Finish when the coral is placed (intake has run for a short time)
    return m_intakeSubsystem->IsIntakeStopped();
}

void PlaceCoralTier4Command::End(bool interrupted) {
    // Stop the intake and lower the lift (if needed)
    m_intakeSubsystem->StopIntake();
    m_liftSubsystem->SetTargetHeight(0.0_m);
}