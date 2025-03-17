#include "RobotContainer.h"
#include <utility>
#include <iostream>
#include <frc/controller/PIDController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include "GlobalConstants.h"

bool RobotContainer::IsBlue() {
  return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
}

frc2::Command* RobotContainer::HandlePartnerCommands(frc2::Command* solo, frc2::Command* partner) {
  return new frc2::InstantCommand(
    [this, solo, partner]() { 
        if(controller2.IsConnected()) partner->Schedule();
        else solo->Schedule();
      }, {});
}

frc2::Command* RobotContainer::GetEmptyCommand() {
  return new frc2::InstantCommand(
          [this]() { 
           }, {});
}

RobotContainer::RobotContainer() {
  autonChooser.SetDefaultOption("None", "None");
  SmartDashboard::PutData(std::move(&autonChooser));

  odomTrigger.WhileTrue(std::move(repeatOdom));

  /*******Controller Bindings*******/
  
  // Intake/Outtake Coral
  controller.A().OnTrue(frc2::cmd::RunOnce([this] { coral.SetIntakePower(1.0); }, {&coral}));
  controller.A().OnFalse(frc2::cmd::RunOnce([this] { coral.SetIntakePower(0.0); }, {&coral}));
  controller.B().OnTrue(frc2::cmd::RunOnce([this] { coral.SetIntakePower(-1.0); }, {&coral}));
  controller.B().OnFalse(frc2::cmd::RunOnce([this] { coral.SetIntakePower(0.0); }, {&coral}));

  // Intake/Outtake Algae
  controller.X().OnTrue(frc2::cmd::RunOnce([this] { algae.SetIntakePower(1.0); }, {&algae}));
  controller.X().OnFalse(frc2::cmd::RunOnce([this] { algae.SetIntakePower(0.0); }, {&algae}));
  controller.Y().OnTrue(frc2::cmd::RunOnce([this] { algae.SetIntakePower(-1.0); }, {&algae}));
  controller.Y().OnFalse(frc2::cmd::RunOnce([this] { algae.SetIntakePower(0.0); }, {&algae}));

  // Raise/Lift Cascade
  controller.LeftBumper().OnTrue(frc2::cmd::RunOnce([this] {
    double currentHeight = cascade.GetPosition().value();
    double newHeight = currentHeight + 0.5; // Raise by 0.5 meters
    if (newHeight > 1.5) newHeight = 1.5; // Cap at 1.5 meters
    cascade.SetTargetPosition(units::meter_t{newHeight});
  }, {&cascade}));

  controller.RightBumper().OnTrue(frc2::cmd::RunOnce([this] {
    double currentHeight = cascade.GetPosition().value();
    double newHeight = currentHeight - 0.5; // Lower by 0.5 meters
    if (newHeight < 0.0) newHeight = 0.0; // Cap at 0 meters
    cascade.SetTargetPosition(units::meter_t{newHeight});
  }, {&cascade}));

  /*******Subsystem DEFAULT Commands*******/

  m_drive.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      double x = -controller.GetLeftY();
      double y = -controller.GetLeftX();
      double turnX = controller.GetRightX();

      if (x > -DriveConstants::kDriveDeadzone && x < DriveConstants::kDriveDeadzone)
          x = 0.0;
      if (y > -DriveConstants::kDriveDeadzone && y < DriveConstants::kDriveDeadzone)
          y = 0.0;

      float xSpeed = DriveConstants::kDriveCurveExtent * pow(x, 3) + (1 - DriveConstants::kDriveCurveExtent) * x;
      float ySpeed = DriveConstants::kDriveCurveExtent * pow(y, 3) + (1 - DriveConstants::kDriveCurveExtent) * y;
      float turn = 0.95 * pow(turnX, 3) + (1 - 0.95) * turnX;

      double cascadeAdjust = 1.0 - (cascade.GetPosition() - CascadeConstants::kStartPosition).value() / 1.2;
      if(cascadeAdjust > 1.0) cascadeAdjust = 1.0;
      if(cascadeAdjust < 0.15) cascadeAdjust = 0.15;
      m_drive.Drive({
        xSpeed * DriveConstants::kDriveTranslationLimit * cascadeAdjust, 
        ySpeed * DriveConstants::kDriveTranslationLimit * cascadeAdjust, 
        turn * -270.0_deg_per_s}, true, fieldCentric);
    }, {&m_drive}));

  algae.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      if(TrackingTarget == GlobalConstants::kAlgaeMode || TrackingTarget == GlobalConstants::kArbitrary) {
        double power = controller.GetLeftTriggerAxis() - (controller.GetRightTriggerAxis()/2);
        if(fabs(power) < 0.1) power = 0.0;
        algae.SetIntakePower(power);
      }
      else {
        algae.SetIntakePower(0.0);
      }
    }, 
  {&algae})); 

  coral.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      if(TrackingTarget == GlobalConstants::kCoralMode || TrackingTarget == GlobalConstants::kArbitrary) {
        double power = (controller.GetLeftTriggerAxis()/2.5) - controller.GetRightTriggerAxis();
        if(fabs(power) < 0.1) power = 0.0;
        coral.SetIntakePower(power);
      }
      else {
        coral.SetIntakePower(0.0);
      }
    }, 
  {&coral}));
}

frc2::CommandPtr RobotContainer::SetAllKinematics(RobotContainer::KinematicsPose pose) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce(
      [this, pose]() {
        cascade.SetTargetPosition(pose.cascadePose);
      }, {&cascade}),
    frc2::cmd::RunOnce(
      [this, pose]() {
        coral.SetTargetAngle(pose.coralAngle);
      }, {&coral})
  );
}

void RobotContainer::SetDriveBrakes(bool state) {
  m_drive.SetBrakeMode(state);
}

void RobotContainer::DisableTagTracking() {
  tagOverrideDisable = true;
  frc::SmartDashboard::PutBoolean("detectorOverride", true);
}

void RobotContainer::EnableTagTracking() {
  tagOverrideDisable = false;
  frc::SmartDashboard::PutBoolean("detectorOverride", false);
}

void RobotContainer::SetSlew(bool state) {
  m_drive.SetLimiting(state);
} 