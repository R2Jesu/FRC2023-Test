#include "Robot.h"

void Robot::R2Jesu_IndexerShooter(void)
{

    
    if (m_Operatorstick.GetR2ButtonPressed())
    {
        m_indexer1.Set(-1);
        m_indexer2.Set(-1);
        m_shooter.Set(-.4);
        isTripped = false;
    }
    else
    {       
        if (!isTripped)
        {
            m_indexer1.Set(1);
            m_indexer2.Set(1);        
        }

        if (indexerPhotocell.Get() && !wasPressed)
        {
            m_indexer1.Set(0);
            m_indexer2.Set(0);
            isTripped=true;
        }

        if (m_Operatorstick.GetCrossButtonPressed())
        {
            wasPressed = true;
            m_indexer1.Set(1);
            m_indexer2.Set(1);
        }

        if (wasPressed && !(indexerPhotocell.Get()))
        {
            wasPressed = false;
            isTripped = false;
        }

        if (m_Operatorstick.GetR1Button())
        {
            desiredRPM = 4000.0;
        }
        else if (m_Operatorstick.GetL2Axis() > 0.0)
        {
            desiredRPM = 2000.0;
        }
        else
        {
            desiredRPM = 0;
        }

        //desiredRPM = 1500.0;
        shooterPidOutput = m_shooterController.Calculate(m_shooterEncoder.GetVelocity(), desiredRPM);
        if (desiredRPM > 0)
        {
            m_shooter.Set(shooterPidOutput);
        }
        else
        {
            m_shooter.Set(0.0);
        }
    }
    //frc::SmartDashboard::PutNumber("desired RPM", desiredRPM);
    //frc::SmartDashboard::PutNumber("shooter velocity", m_shooterEncoder.GetVelocity());
    //frc::SmartDashboard::PutNumber("pid output", shooterPidOutput);
}