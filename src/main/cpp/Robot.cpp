// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

Robot::Robot() {}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

double applyDeadband(double value, double deadband) {
  if (std::abs(value) < deadband) {
    return 0;
  } else {
    return value;
  }
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  double leftY = applyDeadband(m_xbox.GetLeftY(), Constants::CONTROLLER_DEADBAND);
  double leftX = applyDeadband(m_xbox.GetLeftX(), Constants::CONTROLLER_DEADBAND);
  double rightX = applyDeadband(m_xbox.GetRightX(), Constants::CONTROLLER_DEADBAND);

  m_robotPose = m_robotPose + frc::Transform2d{
    frc::Translation2d{units::meter_t{-leftY * 0.02}, units::meter_t{-leftX * 0.02}},
    frc::Rotation2d{units::degree_t{rightX * 0.02}}
  };
  m_leftPose = m_robotPose + Constants::LEFT_CAMERA_TRANSFORM;
  m_rightPose = m_robotPose + Constants::RIGHT_CAMERA_TRANSFORM;
  m_field.SetRobotPose(m_robotPose);
  m_leftPosePub.Set(m_leftPose);
  m_rightPosePub.Set(m_rightPose);
  frc::SmartDashboard::PutData("Robot Field", &m_field);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
