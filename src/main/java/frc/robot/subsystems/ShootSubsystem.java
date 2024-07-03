// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootSubsystem extends SubsystemBase {
  public TalonFX shootingLeftMotor = new TalonFX(22);//
  public TalonFX shootingRightMotor = new TalonFX(23);//
  private CANSparkFlex m_kicker = new CANSparkFlex(24,MotorType.kBrushless);//
  public DigitalInput optic_sensor = new DigitalInput(0);

  private TalonFXConfiguration flyWheelLeftconfig = new TalonFXConfiguration();
  private TalonFXConfiguration flyWheelRightconfig = new TalonFXConfiguration();
  private VelocityVoltage shootingVelDutycycleLeft = new VelocityVoltage(0);
  private VelocityVoltage shootingVelDutycycleRight = new VelocityVoltage(0);


  /** Creates a new ShootingSubsystem. */
  public ShootSubsystem() {
      m_kicker.restoreFactoryDefaults();
      m_kicker.setSmartCurrentLimit(40);
      m_kicker.setIdleMode(IdleMode.kBrake);
      m_kicker.setInverted(true);
      m_kicker.burnFlash();
     /* Motor Inverts and Neutral Mode */
    flyWheelLeftconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flyWheelLeftconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flyWheelLeftconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /* Current Limiting */
    flyWheelLeftconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flyWheelLeftconfig.CurrentLimits.SupplyCurrentLimit = 20;
    flyWheelLeftconfig.CurrentLimits.SupplyCurrentThreshold = 30;
    flyWheelLeftconfig.CurrentLimits.SupplyTimeThreshold = 0.02;

    flyWheelRightconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flyWheelRightconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flyWheelRightconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /* Current Limiting */
    flyWheelRightconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flyWheelRightconfig.CurrentLimits.SupplyCurrentLimit = 20;
    flyWheelRightconfig.CurrentLimits.SupplyCurrentThreshold = 30;
    flyWheelRightconfig.CurrentLimits.SupplyTimeThreshold = 0.02;

     /* Open and Closed Loop Ramping */
    flyWheelLeftconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
    flyWheelLeftconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

    flyWheelLeftconfig.Slot0.kP = 0.6;
    flyWheelLeftconfig.Slot0.kI = 0;
    flyWheelLeftconfig.Slot0.kD = 0;
    flyWheelLeftconfig.Slot0.kV = 0.13;
    flyWheelLeftconfig.Slot0.kS = 0;
    flyWheelLeftconfig.Slot0.kA = 0;
    
    flyWheelRightconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
    flyWheelRightconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

    flyWheelRightconfig.Slot0.kP = 0.6;
    flyWheelRightconfig.Slot0.kI = 0;
    flyWheelRightconfig.Slot0.kD = 0;
    flyWheelRightconfig.Slot0.kV = 0.13;
    flyWheelRightconfig.Slot0.kS = 0;
    flyWheelRightconfig.Slot0.kA = 0;
    
    shootingLeftMotor.getConfigurator().apply(flyWheelLeftconfig);    
    shootingRightMotor.getConfigurator().apply(flyWheelRightconfig);


  }

  public void flywheelControlshoot(double velocity){
    shootingVelDutycycleLeft.Velocity = velocity;
    shootingVelDutycycleRight.Velocity = velocity*0.75;
    shootingLeftMotor.setControl(shootingVelDutycycleLeft);
    shootingRightMotor.setControl(shootingVelDutycycleRight);
    // if(shootingLeftMotor.getVelocity().getValue()>=velocity*0.9)
    // {
    //   Timer.delay(0.4);
    //   m_kicker.set(0.5);
    // }
    // else
    // {
    //   m_kicker.set(0);
    // }                            //0701
    
  }
  public void flywheelControl(double velocity){
    shootingVelDutycycleLeft.Velocity = velocity;
    shootingVelDutycycleRight.Velocity = velocity*0.6;
    shootingLeftMotor.setControl(shootingVelDutycycleLeft);
    shootingRightMotor.setControl(shootingVelDutycycleRight);
  }
  public void trigger_control(double speed)
  {
      if(optic_sensor.get()){
        m_kicker.set(speed);

      }
      else m_kicker.set(0);
  }
  public boolean check()
  {
    return !optic_sensor.get();
  }
  public void standby(){
    if(!optic_sensor.get()){
      flywheelControl(60);
    }
    else flywheelControl(0);
  }
  public void stopkicker(){
    m_kicker.set(0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CAN24",m_kicker.getOutputCurrent());
  }
}
