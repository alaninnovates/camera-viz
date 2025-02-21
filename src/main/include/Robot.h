// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <frc/geometry/Pose2d.h>

#include "Constants.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
 private:
  frc::XboxController m_xbox{0};
  frc::Field2d m_field;
  frc::Pose2d m_robotPose;
  frc::Pose2d m_leftPose, m_rightPose;
  nt::StructPublisher<frc::Pose2d> m_leftPosePub = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/robot/pose/left").Publish();
  nt::StructPublisher<frc::Pose2d> m_rightPosePub = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/robot/pose/right").Publish();
};
