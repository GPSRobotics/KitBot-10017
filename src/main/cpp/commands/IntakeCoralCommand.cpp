// IntakeCoralCommand.cpp
#include "commands/IntakeCoralCommand.h"

IntakeCoralCommand::IntakeCoralCommand(CoralSubsystem* coralSubsystem)
    : m_coralSubsystem{coralSubsystem} {
    AddRequirements(m_coralSubsystem);
}

void IntakeCoralCommand::Initialize() {
    m_coralSubsystem->StartIntake();
}

void IntakeCoralCommand::Execute() {
    // Intake logic (if needed)
}

void IntakeCoralCommand::End(bool interrupted) {
    m_coralSubsystem->StopIntake();
}

bool IntakeCoralCommand::IsFinished() {
    return false; // Run until interrupted
}