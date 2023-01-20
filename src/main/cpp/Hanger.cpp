#include "Robot.h"

void Robot::R2Jesu_Hanger(void)
{
    //m_HangerLeft.Set(.1);
    //m_HangerRight.Set(.1);
    if (m_Drivestick.GetR1Button())
        {
            m_HangerLeft.Set(.4);
            m_HangerRight.Set(.4);
        }
        else if (m_Drivestick.GetL2Axis() > 0.0)
        {
            if(leftLimit.Get())
            {
            m_HangerLeft.Set(-.25);
            } else
            {
                m_HangerLeft.Set(0.0);
            }
            if(rightLimit.Get())
            {
            m_HangerRight.Set(-.25);
            } else
            {
                m_HangerRight.Set(0.0);
            }
        }
        else
        {
            m_HangerLeft.Set(0);
            m_HangerRight.Set(0);
        }
    if (m_Drivestick.GetRightX() > 0.0) 
    {
        hangerPneumatics.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    //go up
    if (m_Drivestick.GetL1Button()) 
    {
        hangerPneumatics.Set(frc::DoubleSolenoid::Value::kForward);
    }
}