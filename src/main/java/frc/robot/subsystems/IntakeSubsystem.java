// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_intake = new CANSparkMax(21,MotorType.kBrushless);
  // public boolean Intake_isinited = false;
  // public boolean taking = false;
  /** Creates a new Intake. */
  public IntakeSubsystem() {
   m_intake.restoreFactoryDefaults();
   m_intake.setInverted(true);
   m_intake.setIdleMode(IdleMode.kCoast);
   m_intake.burnFlash();
  //  Intake_isinited = true;
  }
  public void intake(double speed)
  {
    m_intake.set(speed);
    
    }
    //insert value here
  
  public void autointake(){
    m_intake.set(0.9);
    Timer.delay(2);
    m_intake.set(0);
  }
  public void standby()
  {
    m_intake.set(0.1);
  }
  public void setZero()
  {
    m_intake.set(0);
  }
  
  // public boolean isstandby()
  // {
  //   return !taking;
  // }

  public RelativeEncoder getencoder()
  {
    return m_intake.getEncoder();
  }
  // public Command intakeCommand()
  // {
  //   return this.runOnce(()->autointake());
  // }
   
  @Override
  public void periodic() {
  }
}
