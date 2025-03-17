#pragma once

#include <frc/XboxController.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/DriverStation.h>
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>
#include <frc2/command/Command.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include "units/angle.h"

#include "GlobalConstants.h"

#include "subsystems/CoralSubsystem/CoralSubsystem.h"
#include "subsystems/AlgaeSubsystem/AlgaeSubsystem.h"
#include "subsystems/DriveSubsystem/DriveSubsystem.h"
#include "subsystems/CascadeSubsystem/CascadeSubsystem.h"
#include "subsystems/LEDSubsystem/LEDSubsystem.h"

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include "iostream"
#include "frc/motorcontrol/Spark.h"

class RobotContainer {
 public:
  RobotContainer();

  struct KinematicsPose {
    units::length::meter_t cascadePose;
    units::angle::degree_t coralAngle;
  };

  frc2::CommandPtr SetAllKinematics(KinematicsPose pose);
  frc2::CommandPtr GetAutonomousCommand();
  void SetDriveBrakes(bool state);
  void DisableTagTracking();
  void EnableTagTracking();
  void ZeroSwerve();
  void HandleIntake();
  void SetSlew(bool state);
  void SetRecording(bool state);
  void SetAutoIndex(bool state);

 private:
  frc2::CommandXboxController controller{OIConstants::kDriverControllerPort};
  frc2::CommandXboxController controller2{OIConstants::kCoDriverControllerPort};
  
  int TrackingTarget = GlobalConstants::kCoralMode;
  
  AlgaeSubsystem algae{};
  DriveSubsystem m_drive{&TrackingTarget};
  CascadeSubsystem cascade{};
  CoralSubsystem coral{};

  KinematicsPose startingPose{
    CascadeConstants::kStartPosition,
    CoralConstants::kWristStartAngle + 15_deg
  };

  KinematicsPose L2Pose {
    0.92_m,
    290.09_deg
  };

  KinematicsPose L3Pose {
    1.25_m,
    285.09_deg
  };
  KinematicsPose L4Pose {
    1.79_m,
    152.71_deg
  };
  KinematicsPose L1Pose {
    1.0_m,
    46.03_deg
  };
  KinematicsPose CoralLoad {
    0.92_m,
    107.03_deg
  };
  KinematicsPose L2AlgaeDescore {
    0.92_m,
    250_deg
  };
  KinematicsPose L3AlgaeDescore {
    1.1_m,
    250_deg
  };

  units::degree_t startOffset{180.0};
  bool fieldCentric = true;
  bool omegaOverride = false;
  bool yOverride = false;
  bool validTag = false;
  bool tagOverrideDisable = false;
  bool autoHuntEnabled = false;
  bool autoIntakeEnabled = false;
  int currentTarget = 0;
  int omegaTempDisabled = 0;

  frc2::CommandPtr updateOdometry {
    frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this] {
        m_drive.ResetFromJetson();
      }, {}),
      frc2::cmd::Wait(5.0_s)
    )};

  frc2::CommandPtr autonOdomSet{frc2::cmd::RunOnce([this]{
      m_drive.ResetOdometry(AutoConstants::kDefaultStartingPose);
    },{&m_drive}
  )};

  frc2::CommandPtr odomReset{frc2::cmd::RunOnce([this]{
      m_drive.ResetOdometry({7.5_m, 4.3_m, 180_deg});
  },{})};

  frc2::CommandPtr repeatOdom{std::move(updateOdometry).Repeatedly()};

  frc2::Trigger odomTrigger{[this]() { 
      return false;
    }};

  frc2::CommandPtr toggleFieldCentric{frc2::cmd::RunOnce([this] {
      fieldCentric = !fieldCentric;
    }, {})
  };

  frc2::CommandPtr toggleOmegaOverride{frc2::cmd::RunOnce([this] { 
      omegaOverride = !omegaOverride;
      m_drive.SetOmegaOverride(omegaOverride);
    }, {})
  };

  frc2::Trigger mainDpadUp{controller.POV(0)};
  frc2::Trigger mainDpadDown{controller.POV(180)};
  frc2::Trigger mainDpadLeft{controller.POV(270)};
  frc2::Trigger mainDpadRight{controller.POV(90)};

  frc2::Trigger driverTurning{[this]() {
      return abs(controller.GetRightX()) > DriveConstants::kTurnDeadzone && !controller2.A().Get();
    }
  };

  frc2::CommandPtr targetArbitrary{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kArbitrary;
    }, {})
  };
  
  frc2::CommandPtr targetCoral{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kCoralMode;
    }, {})
  };
  
  frc2::CommandPtr targetAlgae{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kAlgaeMode;
    }, {})
  };

  frc2::CommandPtr driveOff{frc2::cmd::RunOnce([this] { 
      m_drive.Drive({0_mps, 0_mps, 0_deg_per_s});
    }, {&m_drive})
  };

  frc2::CommandPtr autonTrackingDisable{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kArbitrary;
      m_drive.SetOmegaOverride(false);
    }, {&m_drive})  
  };

  frc2::CommandPtr rumblePrimaryOn{frc2::cmd::RunOnce([this] { controller.GetHID().SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {})};

  frc2::CommandPtr rumbleSecondaryOn{frc2::cmd::RunOnce([this] { controller2.GetHID().SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {})};
  
  frc2::CommandPtr rumblePrimaryOff{frc2::cmd::RunOnce([this] { controller.GetHID().SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {})};

  frc2::CommandPtr rumbleSecondaryOff{frc2::cmd::RunOnce([this] { controller2.GetHID().SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {})};

  bool IsBlue();
  frc2::Command* HandlePartnerCommands(frc2::Command* solo, frc2::Command* partner);
  frc2::Command* GetEmptyCommand();
  frc2::CommandPtr SetMultijoint(units::length::meter_t cascadeHeight, units::angle::degree_t algaeAngle);

  frc::SendableChooser<std::string> autonChooser;
  frc2::Command* currentAuton;
};