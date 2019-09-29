/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include<Lib830.h>
#include <frc/IterativeRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include <frc/WPILib.h>

class Robot : public frc::IterativeRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  double Absolute_Multiply(double first_num, double second_num);
  static const int LEFT_FRONT = 3;
  static const int RIGHT_FRONT = 5;
  static const int LEFT_BACK = 1;
  static const int RIGHT_BACK = 6;
  static constexpr double DEADZONE = .1;
  WPI_VictorSPX front_left{LEFT_FRONT};
  WPI_VictorSPX front_right{RIGHT_FRONT};
  WPI_TalonSRX back_right{RIGHT_BACK};
  WPI_TalonSRX back_left{LEFT_BACK};
  frc::SpeedControllerGroup  left{back_left, front_left};
  frc::SpeedControllerGroup  right{back_right, front_right};
  frc::XboxController pilot{0};
  frc::DifferentialDrive drivetrain {right,left};
  frc::GenericHID::JoystickHand Left = frc::GenericHID::kLeftHand;

 private:



 //Controllers

 //Lib830::GamepadF310 pilot {0};

};