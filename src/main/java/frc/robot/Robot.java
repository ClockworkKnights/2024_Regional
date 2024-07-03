// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.Arm_hand_velCmd;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  public Boolean is_loaded = true;
  Boolean is_down = false;
  private long m_starttime = 0;
  private long m_currenttime = 0;
  double hand_position;
  double arm_position;
  
   private CANcoder FL_cancoder = new CANcoder(0);
  private CANcoder FR_cancoder = new CANcoder(1);
  private CANcoder BR_cancoder = new CANcoder(2);
  private CANcoder BL_cancoder = new CANcoder(3);/**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture(0);
    arm_position = m_robotContainer.m_arm.getArmCANPosition();
    hand_position = m_robotContainer.m_arm.getHandCANPosition();
    
    //  m_robotContainer.m_robotDrive.resetEncoders();
    // m_robotContainer.m_robotDrive.zeroHeading();   
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() { 
    // SmartDashboard.putNumber("gyro1", m_robotContainer.m_robotDrive.);
    // SmartDashboard.putNumber("gyro2", m_robotContainer.m_robotDrive.getPose().getX());
    // SmartDashboard.putNumber("gyro3", m_robotContainer.m_robotDrive.getPose().getY());
    // SmartDashboard.putNumber("gyro4", m_robotContainer.m_robotDrive.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("cancoder0",
    FL_cancoder. getAbsolutePosition().getValue() * Math.PI * 2);
SmartDashboard.putNumber("cancoder1",
    FR_cancoder.getAbsolutePosition().getValue() * Math.PI * 2);
SmartDashboard.putNumber("cancoder3",
    BL_cancoder.getAbsolutePosition().getValue() * Math.PI * 2);
SmartDashboard.putNumber("cancoder2",
    BR_cancoder.getAbsolutePosition().getValue() * Math.PI * 2);

    // SmartDashboard.putNumber("arm_pos", m_robotContainer.m_drivestick2.getRawAxis(5));
    // SmartDashboard.putNumber("arm_control",m_robotContainer.arm_pos);
    // SmartDashboard.putNumber("hand_pos", m_robotContainer.m_drivestick2.getRawAxis(1));
    // SmartDashboard.putNumber("hand_control",m_robotContainer.hand_pos);
//  m_robotContainer.arm_pos = Math.abs(m_robotContainer.m_drivestick2.getRawAxis(5))> 0.15 ? -m_robotContainer.m_drivestick2.getRawAxis(5)/10: 0.0;
    // m_robotContainer.hand_pos = Math.abs(m_robotContainer.m_drivestick2.getRawAxis(1))> 0.15 ? -m_robotContainer.m_drivestick2.getRawAxis(1)/10: 0.0;
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    

    if(m_robotContainer.m_shooter.check()){
      is_loaded = false;
    }
    else is_loaded = true;
    if(arm_position-ArmConstants.downarm<=0.1)is_down = true;
    else is_down = false;
    m_currenttime = System.nanoTime();
    SmartDashboard.putBoolean("Go Climb", (m_currenttime-m_starttime)/1000000000 > 115);
    SmartDashboard.putBoolean("Is Loaded", !is_loaded);
    SmartDashboard.putBoolean("Is Down", is_down);
    SmartDashboard.putNumber("Arm",m_robotContainer.m_arm.getArmMotorPosition());
    SmartDashboard.putNumber("Hand",m_robotContainer.m_arm.getHandMotorPosition());
    SmartDashboard.putNumber("Left Velocity",m_robotContainer.m_shooter.shootingLeftMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Right Velocity",m_robotContainer.m_shooter.shootingRightMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Gyro",m_robotContainer.m_robotDrive.getHeading());

  //  System.out.println("ST:"+m_starttime); b7
  //  System.out.println("CR:"+m_currenttime);
    // SmartDashboard.putNumber("shooterL_velocity",m_robotContainer.m_shooter.m_ShooterL.getEncoder().getVelocity());
    // SmartDashboard.putNumber("shooterR_velocity",m_robotContainer.m_shooter.m_ShooterR.getEncoder().getVelocity());
    // SmartDashboard.putNumber("shooterL_current",m_robotContainer.m_shooter.m_ShooterL.getOutputCurrent());
    // SmartDashboard.putNumber("shooterR_current",m_robotContainer.m_shooter.m_ShooterR.getOutputCurrent());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_starttime = System.nanoTime();
    //SmartDashboard.putNumber("Arm",m_robotContainer.m_arm.getArmMotorPosition());
    //SmartDashboard.putNumber("Hand",m_robotContainer.m_arm.getHandMotorPosition());
    // System.out.println(m_robotContainer.m_arm.getinsideEncoder)
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_robotContainer.m_robotDrive.resetEncoders();
    //  m_robotContainer.m_robotDrive.zeroHeading();
    m_robotContainer.m_robotDrive.zeroHeading();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
// if(m_robotContainer.m_robotDrive.getStates()[0].speedMetersPerSecond<=1)
// System.out.println("!!!");
//     m_robotContainer.m_robotDrive.getStates()[0].angle=new Rotation2d(0);
//     if(m_robotContainer.m_robotDrive.getStates()[1].speedMetersPerSecond<=0.1)
//     m_robotContainer.m_robotDrive.getStates()[1].angle=new Rotation2d(0);
//     if(m_robotContainer.m_robotDrive.getStates()[2].speedMetersPerSecond<=0.1)
//     m_robotContainer.m_robotDrive.getStates()[2].angle=new Rotation2d(0);
//     if(m_robotContainer.m_robotDrive.getStates()[3].speedMetersPerSecond<=0.1)
//     m_robotContainer.m_robotDrive.getStates()[3].angle=new Rotation2d(0);
System.out.println("time:"+(m_currenttime-m_starttime)/1000000000);

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_starttime = System.nanoTime();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if(m_robotContainer.hand_pos<0)m_robotContainer.hand_down.schedule();
    // else if(m_robotContainer.hand_pos>0)m_robotContainer.hand_up.schedule();
    //  else {   m_robotContainer.hand_up.cancel();  m_robotContainer.hand_down.cancel(); }
    //  if(m_robotContainer.arm_pos>0)m_robotContainer.arm_up.schedule();
    // else if(m_robotContainer.arm_pos<0)m_robotContainer.arm_down.schedule();
    // else {m_robotContainer.arm_up.cancel();  m_robotContainer.arm_down.cancel(); }

    // if(m_robotContainer.m_drivestick2.getLeftY()>0.05&&m_robotContainer.m_drivestick2.getRightY()>0.05)
    // {
    //       m_robotContainer.m_arm.armsetvelposition(0.05);
    //       m_robotContainer.m_arm.handsetvelposition(0.05);
    // }
    // else if(m_robotContainer.m_drivestick2.getLeftY()>0.05&&m_robotContainer.m_drivestick2.getRightY()<-0.05)
    // {
    //       m_robotContainer.m_arm.armsetvelposition(0.05);
    //       m_robotContainer.m_arm.handsetvelposition(-0.05);
    // }
    // else if(m_robotContainer.m_drivestick2.getLeftY()<-0.05&&m_robotContainer.m_drivestick2.getRightY()>0.05)
    // {
    //       m_robotContainer.m_arm.armsetvelposition(-0.05);
    //       m_robotContainer.m_arm.handsetvelposition(0.05);
    // }
    // else if(m_robotContainer.m_drivestick2.getLeftY()<-0.05&&m_robotContainer.m_drivestick2.getRightY()<-0.05)
    // {
    //       m_robotContainer.m_arm.armsetvelposition(-0.05);
    //       m_robotContainer.m_arm.handsetvelposition(-0.05);
    // } 
    // else
    // {
    //   m_robotContainer.m_arm.armsetvelposition(0);
    //   m_robotContainer.m_arm.handsetvelposition(0);
    // }
    // System.out.println(m_robotContainer.m_drivestick.getL2Button());
    // if(m_robotContainer.m_drivestick.getL2Button())
    // {
    //   m_robotContainer.m_shooter.trigger_control(1);
    // }
    
    //  new JoystickButton(m_robotContainer.m_drivestick, 1).whileTrue(m_robotContainer.hand_down);
    // new JoystickButton(m_robotContainer.m_drivestick, 3).whileTrue(m_robotContainer.hand_up);

    m_currenttime = System.nanoTime();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
