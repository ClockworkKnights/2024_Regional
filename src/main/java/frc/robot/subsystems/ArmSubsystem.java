// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmTestSubsysterm. */
  private CANSparkMax hand_motor;
  private CANcoder hand_encoder;

  private Rotation2d angleOffset;
  private boolean atSetpoint;
  private double setPoint_goal;
  private double acc_now;
  private double feedforward = 0.2;
  private double handoutput;
  private double armoutput;

  public TalonFX m_arm_motorout;//
  public TalonFX m_arm_motorinside;//
  private final PositionVoltage anglePosition = new PositionVoltage(0);
  private CANcoder armCanCoder;

  private ArmFeedforward m_Feedforward = new ArmFeedforward(
      ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  private ProfiledPIDController HandProfiledPIDController = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI,
      0,
      new TrapezoidProfile.Constraints(Constants.ArmConstants.kMaxVelocityRadPerSecond,
          Constants.ArmConstants.kMaxAccelerationRadPerSecSquared));
  private ProfiledPIDController ArmProfiledPIDController = new ProfiledPIDController(1, ArmConstants.kI,//0703
      0,
      new TrapezoidProfile.Constraints(Constants.ArmConstants.kMaxVelocityRadPerSecond,
          Constants.ArmConstants.kMaxAccelerationRadPerSecSquared));

  public ArmSubsystem() {
    // hand
    hand_encoder = new CANcoder(5);
    // hand_encoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

    hand_motor = new CANSparkMax(25, MotorType.kBrushless);
    hand_motor.restoreFactoryDefaults();
    hand_motor.setIdleMode(IdleMode.kBrake);
    hand_motor.setInverted(false);
 
    // hand_motor.setSmartCurrentLimit(40, 100, 6000);

    HandProfiledPIDController.setTolerance(0.01);
    HandProfiledPIDController.reset(getHandCANPosition());
    HandProfiledPIDController.setGoal(getHandCANPosition());
    hand_motor.getEncoder().setPositionConversionFactor(0.05709090);// 1 / 110* 2*Math.PI
    hand_motor.getEncoder().setVelocityConversionFactor(9.5151515e-4);// 1 / 110* 2*Math.PI / 60

    hand_motor.burnFlash();

    // arm
    armCanCoder = new CANcoder(4);
    // armCanCoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

    m_arm_motorout = new TalonFX(26);
    m_arm_motorinside = new TalonFX(27);

    m_arm_motorout.setNeutralMode(NeutralModeValue.Brake);
    m_arm_motorinside.setNeutralMode(NeutralModeValue.Brake);

    m_arm_motorout.setInverted(true);
    m_arm_motorinside.setInverted(false);

    ArmProfiledPIDController.setTolerance(0.01);
    ArmProfiledPIDController.reset(getArmCANPosition());
    ArmProfiledPIDController.setGoal(getArmCANPosition());

    reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("insidemotor position", m_arm_motorinside.getPosition().getValue());
    // SmartDashboard.putNumber("insidemotor speed", m_arm_motorinside.getVelocity().getValue());
    // SmartDashboard.putNumber("insidemotor votage", m_arm_motorinside.getMotorVoltage().getValue());
    // SmartDashboard.putNumber("outmotor votage", m_arm_motorout.getMotorVoltage().getValue());
    // SmartDashboard.putNumber("outmotor position", m_arm_motorout.getPosition().getValue());
    // SmartDashboard.putNumber("outmotor speed", m_arm_motorout.getVelocity().getValue());
    SmartDashboard.putNumber("ARMCanCoder", getArmCANPosition());
    SmartDashboard.putNumber("HandCanCoder", getHandCANPosition());
    // SmartDashboard.putNumber("CanRaw",
    // -(_encoder.getAbsolutePosition().getValue() * Math.PI * 2));

  }

  public void setHandPoint(double goal) {

    // if (goal > 6.47)
    //   goal = 6.47;
    // if (goal < 4.0)
    //   goal = 4.0;
    HandProfiledPIDController.setGoal(goal);
    // SmartDashboard.putNumber("handgoal", goal);
  }

  public void setArmPoint(double goal) {

    if (goal > 3.3){
      goal = 3.3;
    }
    if (goal < 2.1){
      goal = 2.1;
    }
    ArmProfiledPIDController.setGoal(goal);
   // SmartDashboard.putNumber("armgoal", goal);
  }

  public double gethandpoint() {
    return HandProfiledPIDController.getGoal().position;
  }

  public double getArmPoint() {
    return ArmProfiledPIDController.getGoal().position;
  }
  // public double getArmcan(){
  //   return armCanCoder.get
  // }

  // public void armupdate() {
  //   // handoutput = feedforward + HandProfiledPIDController.calculate(getHandCANPosition());
  //   // hand_motor.setVoltage(handoutput);

  //   armoutput = ArmProfiledPIDController.calculate(getArmCANPosition());
  //   _setArmVoltage(armoutput);

  //   // SmartDashboard.putNumber("hand output:", handoutput);
  //   // SmartDashboard.putNumber("hand error", HandProfiledPIDController.getPositionError());

  //   SmartDashboard.putNumber("arm output:", armoutput);
  //   SmartDashboard.putNumber("arm error", ArmProfiledPIDController.getPositionError());
  // }
  public void handupdate() {
    handoutput = feedforward + HandProfiledPIDController.calculate(getHandCANPosition());
    hand_motor.setVoltage(handoutput);

    armoutput = ArmProfiledPIDController.calculate(getArmCANPosition());
    _setArmVoltage(armoutput);

    // SmartDashboard.putNumber("hand output:", handoutput);
    // SmartDashboard.putNumber("hand error", HandProfiledPIDController.getPositionError());

    //SmartDashboard.putNumber("arm output:", armoutput);
    //SmartDashboard.putNumber("arm error", ArmProfiledPIDController.getPositionError());
  }

  public void reset() {
    hand_motor.getEncoder().setPosition(getHandCANPosition());
    HandProfiledPIDController.reset(getHandCANPosition());
    setHandPoint(getHandCANPosition());

    m_arm_motorout.setPosition(getArmCANPosition());
    m_arm_motorinside.setPosition(getArmCANPosition());
    //ArmProfiledPIDController.reset(getArmCANPosition());
    setArmPoint(getArmCANPosition());
  }

  public double getHandMotorPosition() {
    return hand_motor.getEncoder().getPosition();
  }

  public double getArmMotorPosition() {
    // return m_arm_motorinside.getPosition().getValue();
    return m_arm_motorout.getPosition().getValue();
  }

  public double getHandCANPosition() {
    if (-hand_encoder.getAbsolutePosition().getValue() * Math.PI * 2 < -3)
      return -hand_encoder.getAbsolutePosition().getValue() * Math.PI * 2 + 4 * Math.PI;
    else
      return -hand_encoder.getAbsolutePosition().getValue() * Math.PI * 2 + 2 * Math.PI;
  }

  public double getArmCANPosition() {
    // if (armCanCoder.getAbsolutePosition().getValue() * Math.PI * 2 < 3.15) {
    //   return armCanCoder.getAbsolutePosition().getValue() * Math.PI * 2 + 2 * Math.PI;
    // } else {
      return armCanCoder.getAbsolutePosition().getValue() * Math.PI * 2;
    //}
  }

  public void _setArmVoltage(double voltage) {
    m_arm_motorinside.setVoltage(-voltage);
    m_arm_motorout.setVoltage(-voltage);
  }

  public void stopMotor() {
    hand_motor.setVoltage(0);
    HandProfiledPIDController.reset(getHandCANPosition());
    HandProfiledPIDController.setGoal(getHandCANPosition());

    m_arm_motorinside.setVoltage(0);
    m_arm_motorout.setVoltage(0);
    ArmProfiledPIDController.reset(getArmCANPosition());
    ArmProfiledPIDController.setGoal(getArmCANPosition());
  }
  public void stoparmmotor(){
        m_arm_motorinside.setVoltage(0);
    m_arm_motorout.setVoltage(0);
    ArmProfiledPIDController.reset(getArmCANPosition());
    ArmProfiledPIDController.setGoal(getArmCANPosition());
  }
  public void stophandmotor(){
     hand_motor.setVoltage(0);
    HandProfiledPIDController.reset(getHandCANPosition());
    HandProfiledPIDController.setGoal(getHandCANPosition());

  }
  public void setposition(double armposition, double handposition){
    armoutput = ArmProfiledPIDController.calculate(getArmCANPosition());
    _setArmVoltage(armoutput);
     handoutput = feedforward + HandProfiledPIDController.calculate(getHandCANPosition());
    hand_motor.setVoltage(handoutput);
    setArmPoint(armposition);
    setHandPoint(handposition);
  }
  public void setarmposition(double armposition){
    armoutput = ArmProfiledPIDController.calculate(getArmCANPosition());
    _setArmVoltage(armoutput);
    setArmPoint(armposition);
    
  }
  public void armsetvelposition(double armposition){
    armoutput = ArmProfiledPIDController.calculate(getArmCANPosition());
    _setArmVoltage(armoutput);
      double position = getArmPoint();
      setArmPoint(position+armposition);
  }
  public void handsetvelposition(double handposition){
     handoutput = feedforward + HandProfiledPIDController.calculate(getHandCANPosition());
    hand_motor.setVoltage(handoutput);
      double position = gethandpoint();
      setHandPoint(position+handposition);

  }
  public void hand_armsetvelposition(double armposition, double handposition){
     handoutput = feedforward + HandProfiledPIDController.calculate(getHandCANPosition());
    hand_motor.setVoltage(handoutput);
      double position = gethandpoint();
      armoutput = ArmProfiledPIDController.calculate(getArmCANPosition());
    _setArmVoltage(armoutput);
      double position1 = getArmPoint();
      setArmPoint(position1+armposition);
      setHandPoint(position+handposition);

  }


  // public void suckin(){
  //   setArmPoint(5.471709470388082);
  //   setHandPoint(5.488583259054824);
  // }

  //   public void AMP(){
  //   setArmPoint(6.16506878651);
  //   setHandPoint(5.75242795457);
  // }
  // public void speaker(){
  //   setArmPoint(5.63891337626);
  //   setHandPoint(5.54994249057025);
  //   // setHandPoint(-0.087382568312866)
    
  // }
  // public void threehundred(){
  //   setArmPoint(5.83372893632909);
  //   setHandPoint(5.611301722085676);
  //   // setHandPoint(-0.087382568312866)
    
  // }
  // public void twohundredfifty(){
  //   setArmPoint(5.81225320529869);
  //   setHandPoint(5.611301722085676);
  //   // setHandPoint(-0.087382568312866)
    
  // }
  // public void twohundredthirty(){
  //   setArmPoint(5.80918524372292);
  //   setHandPoint(5.611301722085676);
  //   // setHandPoint(-0.087382568312866)
    
  // }
  // public void twohundred(){
  //   setArmPoint(5.79077747426829);
  //   setHandPoint(5.611301722085676);
  //   // setHandPoint(-0.087382568312866)
    
  // }
  // public void onehundredfifty(){
  //   setArmPoint(5.77390368560155);
  //   setHandPoint(5.611301722085676);
  //   // setHandPoint(-0.087382568312866)
    
  // }
  // public void onehundred(){
  //   setArmPoint(5.64124347232818);
  //   setHandPoint(5.611301722085676);
  //   // setHandPoint(-0.087382568312866)
    
  // }
  
}
