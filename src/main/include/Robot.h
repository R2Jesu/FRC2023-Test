// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PS4Controller.h>
#include <frc/controller/PIDController.h>
#include <frc/motorcontrol/Spark.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <rev/CANEncoder.h>
#include <rev/CANAnalog.h>
#include <ctre/Phoenix.h>
#include <cmath>
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsModuleType.h>
#include <chrono>
#include <thread>
#include <cameraserver/CameraServer.h>
#include <frc/DriverStation.h>
#include <cscore_oo.h>
#include "AHRS.h"
#include <frc/apriltag/AprilTagDetector.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  void R2Jesu_Drive(void);
  void R2Jesu_Intake(void);
  void R2Jesu_Hanger(void);
  void R2Jesu_IndexerShooter(void);
  void R2Jesu_Autonomous(void);
  void R2Jesu_AutonomousGetOut(void);
  void R2Jesu_AutonomousWaitGetOut(void);
  void R2Jesu_AutonomousShootGetOut(void);
  void R2Jesu_AutonomousSelfish(void);
  void R2Jesu_AutonomousGRC(void);
  void R2Jesu_SwitchAuto(void);
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoOriginal = "Original Auto";
  const std::string kAutoGetOut = "Get Out";
  const std::string kAutoShootGetOut = "Shoot and Get Out";
  const std::string kAutoWaitGetOut = "Wait and Get Out";
  const std::string kAutoSelfish = "Selfish Auto";
  const std::string kAutoGRC = "GRC Auto";
  const std::string kSwitchAuto = "Switch Auto";
  std::string m_autoSelected;
  frc::Compressor compressorObject{frc::PneumaticsModuleType::CTREPCM};
  
  // Swerve Drive 
  frc::PS4Controller m_Drivestick{0};
  frc::PS4Controller m_Operatorstick{1};
  
  //swerve 1 front left
  rev::CANSparkMax m_SwerveDrive1{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder m_DriveEncoder1 = m_SwerveDrive1.GetEncoder();
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_SwerveTurn1{1};
  frc::AnalogInput m_SwerveAnalog1{0};
  
  //swerve 2
  rev::CANSparkMax m_SwerveDrive2{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder m_DriveEncoder2 = m_SwerveDrive2.GetEncoder();
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_SwerveTurn2{2};
  frc::AnalogInput m_SwerveAnalog2{1};
  
  //swerve 3
  rev::CANSparkMax m_SwerveDrive3{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder m_DriveEncoder3 = m_SwerveDrive3.GetEncoder();
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_SwerveTurn3{3};
  frc::AnalogInput m_SwerveAnalog3{2};
  
  //swerve 4
  rev::CANSparkMax m_SwerveDrive4{8, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder m_DriveEncoder4 = m_SwerveDrive4.GetEncoder();
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_SwerveTurn4{4};
  frc::AnalogInput m_SwerveAnalog4{3};

  //Swerve control variables
  double conversion = 360.0/3.3;
  double conversion1 = 360.0/3.235;
  double conversion2 = 360.0/3.265;
  double conversion3 = 360.0/3.275;
  double conversion4 = 360.0/3.275;
  double x=0.0, y=0.0, z=0.0;
  double inputAngle = 0.0;
  double r = 0.0;
  double newX = 0.0, newY = 0.0;
  double fieldOrientedAngle = 0.0;
  double correctionPID;
  double LENGTH = 17.375;
  double WIDTH = 21.25;
  double A=0.0;
	double B=0.0;
	double C=0.0;
	double D=0.0;
  double wSpeed1=0.0;
  double wAngle1=0.0;
  double wSpeed2=0.0;
  double wAngle2=0.0;
  double wSpeed3=0.0;
  double wAngle3=0.0;
  double wSpeed4=0.0;
  double wAngle4=0.0;
  double R = sqrt((LENGTH*LENGTH) + (WIDTH*WIDTH));
  double Ppid = 0.050;//45;
  double Ipid = 0.000;
  double Dpid = 0.001;//0.0008;//.0005
  double switchPpid = 0.025;
  double switchIpid = 0.0;
  double switchDpid = 0.0;
  double pidOutput1 = 0.0;
  double pidOutput2 = 0.0;
  double pidOutput3 = 0.0;
  double pidOutput4 = 0.0;
  double switchPidOutput = 0.0;
  frc2::PIDController m_angleController1{ Ppid , Ipid, Dpid, 20_ms};
  frc2::PIDController m_angleController2{ Ppid , Ipid, Dpid, 20_ms};
  frc2::PIDController m_angleController3{ Ppid , Ipid, Dpid, 20_ms};
  frc2::PIDController m_angleController4{ Ppid , Ipid, Dpid, 20_ms};
  frc2::PIDController m_switchController{ switchPpid, switchIpid, switchDpid, 20_ms};
  double fullSpeed = .3;
  double turnSpeed = .2;
  double speedChoice;
  //intake
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intake{12};
  frc::DoubleSolenoid intakePneumatics{frc::PneumaticsModuleType::CTREPCM,4,5};

  //Indexer
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_indexer1{10};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_indexer2{11};
  frc::DigitalInput indexerPhotocell{9};
  bool isTripped = false;
  bool wasPressed = false;

  //Shooter
  rev::CANSparkMax m_shooter{9, rev::CANSparkMax::MotorType::kBrushless};
  double shooterPpid = 0.00030;
  double shooterIpid = 0.0005;
  double shooterDpid = 0.0000;
  frc2::PIDController m_shooterController{ shooterPpid , shooterIpid, shooterDpid, 20_ms};
  rev::SparkMaxRelativeEncoder m_shooterEncoder = m_shooter.GetEncoder();
  double shooterPidOutput = 0.0;
  double desiredRPM = 0.0;

  //Hanger
  rev::CANSparkMax m_HangerLeft{13, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_HangerRight{14, rev::CANSparkMax::MotorType::kBrushless};
  frc::DoubleSolenoid hangerPneumatics{frc::PneumaticsModuleType::CTREPCM, 6, 7};
  frc::DigitalInput leftLimit{7};
  frc::DigitalInput rightLimit{8};

  //autonomous
  double autoSpeed = .15;
  double shSpeed = 0.0;

  //camera
  cs::UsbCamera drvCamera;

  //NavX
  AHRS *ahrs;
  
};
