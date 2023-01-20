#include "Robot.h"

void Robot::R2Jesu_AutonomousSelfish(void)
{
    intakePneumatics.Set(frc::DoubleSolenoid::Value::kForward);
    m_intake.Set(0.5);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    while (m_DriveEncoder1.GetPosition() < 40.0 && frc::DriverStation::IsAutonomousEnabled())
    {
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 90.0);
        m_SwerveTurn1.Set(pidOutput1);
        m_SwerveDrive1.Set(autoSpeed);
        frc::SmartDashboard::PutNumber("Wheel 1 Voltage", m_SwerveAnalog1.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 1 Angle", (m_SwerveAnalog1.GetVoltage()*conversion1));

        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 87.0);
        m_SwerveTurn2.Set(pidOutput2);
        m_SwerveDrive2.Set(autoSpeed);
        frc::SmartDashboard::PutNumber("Wheel 2 Voltage", m_SwerveAnalog2.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 2 Angle", (m_SwerveAnalog2.GetVoltage()*conversion2));

        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 87.0);
        m_SwerveTurn3.Set(pidOutput3);
        m_SwerveDrive3.Set(autoSpeed);
        frc::SmartDashboard::PutNumber("Wheel 3 Voltage", m_SwerveAnalog3.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 3 Angle", (m_SwerveAnalog3.GetVoltage()*conversion3));

        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 90.0);
        m_SwerveTurn4.Set(pidOutput4);
        m_SwerveDrive4.Set(autoSpeed);
        frc::SmartDashboard::PutNumber("Wheel 4 Voltage", m_SwerveAnalog4.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 4 Angle", (m_SwerveAnalog4.GetVoltage()*conversion4));
        //frc::SmartDashboard::PutNumber("position", m_DriveEncoder1.GetPosition());
    }
    m_SwerveDrive1.Set(0.0);
    m_SwerveDrive2.Set(0.0);
    m_SwerveDrive3.Set(0.0);
    m_SwerveDrive4.Set(0.0);
    m_SwerveTurn1.Set(0.0);
    m_SwerveTurn2.Set(0.0);
    m_SwerveTurn3.Set(0.0);
    m_SwerveTurn4.Set(0.0);
    m_DriveEncoder1.SetPosition(0.0);
    m_DriveEncoder2.SetPosition(0.0);
    m_DriveEncoder3.SetPosition(0.0);
    m_DriveEncoder4.SetPosition(0.0);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    m_intake.Set(0);
    intakePneumatics.Set(frc::DoubleSolenoid::Value::kReverse);

    

    while (m_DriveEncoder1.GetPosition() < 28.0 && frc::DriverStation::IsAutonomousEnabled())
    {
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 275.0);
        m_SwerveTurn1.Set(pidOutput1);
        m_SwerveDrive1.Set(autoSpeed);
        frc::SmartDashboard::PutNumber("Wheel 1 Voltage", m_SwerveAnalog1.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 1 Angle", (m_SwerveAnalog1.GetVoltage()*conversion1));

        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 270.0); //265
        m_SwerveTurn2.Set(pidOutput2);
        m_SwerveDrive2.Set(autoSpeed);
        frc::SmartDashboard::PutNumber("Wheel 2 Voltage", m_SwerveAnalog2.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 2 Angle", (m_SwerveAnalog2.GetVoltage()*conversion2));

        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 270.0); //265
        m_SwerveTurn3.Set(pidOutput3);
        m_SwerveDrive3.Set(autoSpeed);
        frc::SmartDashboard::PutNumber("Wheel 3 Voltage", m_SwerveAnalog3.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 3 Angle", (m_SwerveAnalog3.GetVoltage()*conversion3));

        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 275.0);
        m_SwerveTurn4.Set(pidOutput4);
        m_SwerveDrive4.Set(autoSpeed);
        frc::SmartDashboard::PutNumber("Wheel 4 Voltage", m_SwerveAnalog4.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 4 Angle", (m_SwerveAnalog4.GetVoltage()*conversion4));

    }

    m_SwerveDrive1.Set(0.0);
    m_SwerveDrive2.Set(0.0);
    m_SwerveDrive3.Set(0.0);
    m_SwerveDrive4.Set(0.0);
    m_SwerveTurn1.Set(0.0);
    m_SwerveTurn2.Set(0.0);
    m_SwerveTurn3.Set(0.0);
    m_SwerveTurn4.Set(0.0);
    m_DriveEncoder1.SetPosition(0.0);
    m_DriveEncoder2.SetPosition(0.0);
    m_DriveEncoder3.SetPosition(0.0);
    m_DriveEncoder4.SetPosition(0.0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    while (m_DriveEncoder1.GetPosition() < 34.0 && frc::DriverStation::IsAutonomousEnabled())
    {
        m_SwerveDrive1.Set(autoSpeed);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 270.0);
        m_SwerveTurn1.Set(pidOutput1);
        frc::SmartDashboard::PutNumber("Wheel 1 Voltage", m_SwerveAnalog1.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 1 Angle", (m_SwerveAnalog1.GetVoltage()*conversion1));

        m_SwerveDrive2.Set(autoSpeed);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 275.0);
        m_SwerveTurn2.Set(pidOutput2);
        frc::SmartDashboard::PutNumber("Wheel 2 Voltage", m_SwerveAnalog2.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 2 Angle", (m_SwerveAnalog2.GetVoltage()*conversion2));

        m_SwerveDrive3.Set(autoSpeed);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 275.0);
        m_SwerveTurn3.Set(pidOutput3);
        frc::SmartDashboard::PutNumber("Wheel 3 Voltage", m_SwerveAnalog3.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 3 Angle", (m_SwerveAnalog3.GetVoltage()*conversion3));

        m_SwerveDrive4.Set(autoSpeed);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 270.0);
        m_SwerveTurn4.Set(pidOutput4);
        frc::SmartDashboard::PutNumber("Wheel 4 Voltage", m_SwerveAnalog4.GetVoltage());
        frc::SmartDashboard::PutNumber("Wheel 4 Angle", (m_SwerveAnalog4.GetVoltage()*conversion4));

    }

    m_SwerveDrive1.Set(0.0);
    m_SwerveDrive2.Set(0.0);
    m_SwerveDrive3.Set(0.0);
    m_SwerveDrive4.Set(0.0);
    m_SwerveTurn1.Set(0.0);
    m_SwerveTurn2.Set(0.0);
    m_SwerveTurn3.Set(0.0);
    m_SwerveTurn4.Set(0.0);
    m_DriveEncoder1.SetPosition(0.0);
    m_DriveEncoder2.SetPosition(0.0);
    m_DriveEncoder3.SetPosition(0.0);
    m_DriveEncoder4.SetPosition(0.0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    desiredRPM = 910.0;
    shSpeed = 0.001;
    while ((m_shooterEncoder.GetVelocity() < desiredRPM) && frc::DriverStation::IsAutonomousEnabled())
    {
        shSpeed = shSpeed + 0.0001;
        m_shooter.Set(shSpeed);
        //printf("velo in while %f", m_shooterEncoder.GetVelocity());
    }
    m_indexer1.Set(1.0);
    m_indexer2.Set(1.0);
    
}