#include "commands/IntakeCoralCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

IntakeCoralCommand::IntakeCoralCommand(IntakeSubsystem* intakeSubsystem, UltrasonicSensor* ultrasonicSensor)
    : m_intakeSubsystem(intakeSubsystem), m_ultrasonicSensor(ultrasonicSensor) {
    AddRequirements(m_intakeSubsystem);
}

void IntakeCoralCommand::Initialize() {
    // Start the intake motor
    m_intakeSubsystem->StartIntake();
}

void IntakeCoralCommand::Execute() {
    // Check the ultrasonic sensor distance
    double distance = m_ultrasonicSensor->GetDistance();
    frc::SmartDashboard::PutNumber("Ultrasonic Distance", distance);

    // Stop if the coral is detected (e.g., distance < 10 cm)
    if (distance < 10.0) {
        m_intakeSubsystem->StopIntake();
    }
}

bool IntakeCoralCommand::IsFinished() {
    // Finish when the coral is detected and intake is stopped
    return m_intakeSubsystem->IsIntakeStopped();
}

void IntakeCoralCommand::End(bool interrupted) {
    // Ensure the intake is stopped
    m_intakeSubsystem->StopIntake();
}