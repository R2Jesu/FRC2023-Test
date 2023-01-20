#include "Robot.h"

void Robot::R2Jesu_Intake(void)
{
    if (m_Operatorstick.GetL1Button()) 
    {
        intakePneumatics.Set(frc::DoubleSolenoid::Value::kReverse);
        m_intake.Set(0.0);
    }
//go up
    if (m_Operatorstick.GetRightX() > 0.0) 
    {
        intakePneumatics.Set(frc::DoubleSolenoid::Value::kForward);
        m_intake.Set(0.5);
    }
}
