#include "Robot.h"

void Robot::R2Jesu_AutonomousShootGetOut(void)
{
    printf("auto selected: shoot get out");
    desiredRPM = 910.0;
    shSpeed = 0.001;
    while ((m_shooterEncoder.GetVelocity() < desiredRPM) && frc::DriverStation::IsAutonomousEnabled())
    {
        shSpeed = shSpeed + 0.0001;
        m_shooter.Set(shSpeed);
    }
    m_indexer1.Set(1.0);
    m_indexer2.Set(1.0);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    while (m_DriveEncoder1.GetPosition() < 43.0 && frc::DriverStation::IsAutonomousEnabled())
    {
        m_SwerveDrive1.Set(autoSpeed);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 90.0);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(autoSpeed);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 90.0);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(autoSpeed);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 90.0);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(autoSpeed);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 90.0);
        m_SwerveTurn4.Set(pidOutput4);
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

}