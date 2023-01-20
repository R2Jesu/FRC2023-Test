#include "Robot.h"

void Robot::R2Jesu_AutonomousWaitGetOut(void)
{
    printf("auto selected: wait get out");
    std::this_thread::sleep_for(std::chrono::seconds(10));
    printf("waiting over\n");
    while (m_DriveEncoder1.GetPosition() < 48.0 && frc::DriverStation::IsAutonomousEnabled())
    {
        printf("In while\n");
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