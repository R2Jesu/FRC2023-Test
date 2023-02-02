// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.comp
#include "Robot.h"

void Robot::RobotInit() {
  ahrs = new AHRS(frc::SPI::Port::kMXP);

  m_chooser.SetDefaultOption(kAutoOriginal, kAutoOriginal);
  m_chooser.AddOption(kAutoGetOut, kAutoGetOut);
  m_chooser.AddOption(kAutoWaitGetOut, kAutoWaitGetOut);
  m_chooser.AddOption(kAutoShootGetOut, kAutoShootGetOut);
  m_chooser.AddOption(kAutoSelfish, kAutoSelfish);
  m_chooser.AddOption(kAutoGRC, kAutoGRC);
  m_chooser.AddOption(kSwitchAuto, kSwitchAuto);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_angleController1.EnableContinuousInput(0.00, 360.00);
  m_angleController2.EnableContinuousInput(0.00, 360.00);
  m_angleController3.EnableContinuousInput(0.00, 360.00);
  m_angleController4.EnableContinuousInput(0.00, 360.00);

  //frc::SmartDashboard::PutNumber("Ppid",Ppid);
  //frc::SmartDashboard::PutNumber("Ipid",Ipid);
  //frc::SmartDashboard::PutNumber("Dpid",Dpid);
  isTripped = false;
  wasPressed = false;

  compressorObject.EnableDigital();
  m_indexer1.Set(0.0);
  m_indexer2.Set(0.0);
  m_shooter.Set(0.0);

  //camera
  drvCamera = frc::CameraServer::StartAutomaticCapture();
  drvCamera.SetResolution(320, 240);
  drvCamera.SetFPS(15);
  drvCamera.SetExposureManual(40);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

//Ppid = frc::SmartDashboard::GetNumber("Ppid",0.0);
//Ipid = frc::SmartDashboard::GetNumber("Ipid",0.0);
//Dpid = frc::SmartDashboard::GetNumber("Dpid",0.0);
/*if (m_Operatorstick.GetR2Axis() > 0.0)
{
  printf("R2axis active\n");
}
if (m_Operatorstick.GetR2ButtonPressed())
{
  printf("R2Button active\n");
}
if (m_Operatorstick.GetR3ButtonPressed())
{
  printf("R3Button active\n");
}
if (m_Operatorstick.GetRightX() > 0.0)
{
  printf("RightX active\n");
}
if (m_Operatorstick.GetRightY() > 0.0)
{
  printf("RightY active\n");
}
if (m_Operatorstick.GetShareButtonPressed())
{
  printf("Share Active\n");
}
if (m_Operatorstick.GetSquareButtonPressed())
{
  printf("Square active\n");
}
if (m_Operatorstick.GetTouchpadPressed())
{
  printf("touchpad active\n");
}
if (m_Operatorstick.GetTriangleButtonPressed())
{
  printf("Triangle active\n");
}
if (m_Operatorstick.GetCircleButtonPressed())
{
  printf("Circle active\n");
}
if (m_Operatorstick.GetCrossButtonPressed())
{
  printf("Cross active\n");
}
if (m_Operatorstick.GetL1ButtonPressed())
{
  printf("L1 active\n");
}
if (m_Operatorstick.GetL2ButtonPressed())
{
  printf("L2 active\n");
}
if (m_Operatorstick.GetL3ButtonPressed())
{
  printf("L3 active\n");
}
if (m_Operatorstick.GetOptionsButtonPressed())
{
  printf("Options active\n");
}
if (m_Operatorstick.GetPSButtonPressed())
{
  printf("PSbutton active\n");
}
if (m_Operatorstick.GetR1ButtonPressed())
{
  printf("R1 active\n");
}
if (m_Operatorstick.GetLeftX() > 0.0)
{
  printf("LeftX active\n");
}
if (m_Operatorstick.GetLeftY() > 0.0)
{
  printf("LeftY active\n");
}
if (m_Operatorstick.GetL2Axis() > 0.0)
{
  printf("L2Axis active\n");
}
if (m_Operatorstick.GetRightX() > 0.0)
{
  printf("RightX Active\n");
}
if (m_Operatorstick.GetRightY() > 0.0)
{
  printf("RightY active\n");
}*/


}


/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {

  ahrs->ResetDisplacement();
  m_indexer1.Set(0.0);
  m_indexer2.Set(0.0);
  m_shooter.Set(0.0);
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  //printf("Auto selected: %s\n", m_autoSelected);

  //if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  //} else {
    // Default Auto goes here
  //}
  m_DriveEncoder1.SetPosition(0.0);
  m_DriveEncoder2.SetPosition(0.0);
  m_DriveEncoder3.SetPosition(0.0);
  m_DriveEncoder4.SetPosition(0.0);
  m_SwerveDrive1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_SwerveDrive2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_SwerveDrive3.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_SwerveDrive4.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
/*   if (m_autoSelected == kAutoOriginal)
  {
    R2Jesu_Autonomous();
  }
  else if (m_autoSelected == kAutoGetOut)
  {
    R2Jesu_AutonomousGetOut();
  }
  else if (m_autoSelected == kAutoWaitGetOut)
  {
    R2Jesu_AutonomousWaitGetOut();
  }
  else if (m_autoSelected == kAutoShootGetOut)
  {
    R2Jesu_AutonomousShootGetOut();
  }
  else if (m_autoSelected == kAutoSelfish)
  {
    R2Jesu_AutonomousSelfish();
  }
  else if (m_autoSelected == kAutoGRC)
  {
    R2Jesu_AutonomousGRC();
  }
  else if (m_autoSelected == kSwitchAuto)
  {
    R2Jesu_SwitchAuto();
  } 
  else
  {
    R2Jesu_Autonomous();
  } */
  
}

void Robot::AutonomousPeriodic() {
  R2Jesu_SwitchAuto();
}

void Robot::TeleopInit() 
{
  //printf("Tele Init\n");
  m_indexer1.Set(0.0);
  m_indexer2.Set(0.0);
  m_shooter.Set(0.0);
  desiredRPM = 0.0;
    isTripped = false;
    wasPressed = false;
    m_SwerveDrive1.Set(0.0);
    m_SwerveDrive2.Set(0.0);
    m_SwerveDrive3.Set(0.0);
    m_SwerveDrive4.Set(0.0);
    m_SwerveTurn1.Set(0.0);
    m_SwerveTurn2.Set(0.0);
    m_SwerveTurn3.Set(0.0);
    m_SwerveTurn4.Set(0.0);
  m_SwerveDrive1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_SwerveDrive2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_SwerveDrive3.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_SwerveDrive4.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

}

void Robot::TeleopPeriodic() 
{
  //printf("Tele Periodic\n");
  R2Jesu_Intake();
  R2Jesu_Drive();
  R2Jesu_Hanger();
  R2Jesu_IndexerShooter();
  frc::SmartDashboard::PutNumber("shooter speed", m_shooterEncoder.GetVelocity());
  //frc::SmartDashboard::PutNumber("speed2", wSpeed2);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
