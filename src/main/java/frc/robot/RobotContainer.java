// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Auto.AutoCmd;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.Arm_hand_velCmd;
import frc.robot.commands.ArmdownCmd;
import frc.robot.commands.ArmvelCmd;
import frc.robot.Auto.AutoIntakeCmd;
import frc.robot.Auto.autoleft;
import frc.robot.Auto.automiddle;
import frc.robot.Auto.autoright;
import frc.robot.Auto.onlymove;
import frc.robot.Auto.onlyshoot;
import frc.robot.Auto.threeautoleft;
import frc.robot.Auto.threeautoright;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.HandvelCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.Shoot_standby;
import frc.robot.commands.TriggerCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.commands.OnlyShootCmd;
import frc.robot.commands.Reset;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Arm;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import java.util.HashMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //subsystems:  
    public static DriveSubsystem m_robotDrive = new DriveSubsystem();
    public static ArmSubsystem m_arm = new ArmSubsystem();
    public static IntakeSubsystem m_intake = new IntakeSubsystem();
    public static ShootSubsystem m_shooter = new ShootSubsystem();

    public ArmCmd armsuckin = new ArmCmd(m_arm, ArmConstants.suckinarm, ArmConstants.suckinhand);
    public ArmCmd armsuckinspit = new ArmCmd(m_arm, ArmConstants.suckinarm, ArmConstants.suckinhand);
    public ArmCmd armfirst = new ArmCmd(m_arm, ArmConstants.firstarm, ArmConstants.firsthand);
    public ArmCmd armsecond = new ArmCmd(m_arm, ArmConstants.secondarm, ArmConstants.secondhand);
    public ArmCmd armdown = new ArmCmd(m_arm, ArmConstants.downarm, ArmConstants.downhand);
    public ArmCmd armamp = new ArmCmd(m_arm, ArmConstants.amparm, ArmConstants.amphand);
    public ArmCmd armclimb = new ArmCmd(m_arm, ArmConstants.climbarm, ArmConstants.climbhand);
    public ArmCmd armtrap = new ArmCmd(m_arm, ArmConstants.traparm, ArmConstants.traphand);
    public ArmCmd armsecondhigh = new ArmCmd(m_arm, ArmConstants.secondhigharm, ArmConstants.secondhighhand);
    public ArmCmd armthird = new ArmCmd(m_arm, ArmConstants.thirdarm, ArmConstants.thirdhand);
    public ArmCmd armthirdhigh = new ArmCmd(m_arm, ArmConstants.thirdhigharm, ArmConstants.thirdhighhand);
    public ArmCmd armfourth = new ArmCmd(m_arm, ArmConstants.fourtharm, ArmConstants.fourthhand);
    public ArmCmd armpass = new ArmCmd(m_arm, ArmConstants.passarm, ArmConstants.passhand);
    // public ArmdownCmd downarm = new ArmdownCmd(m_arm, ArmConstants.downarm, ArmConstants.downhand);

    public ArmvelCmd arm_up = new ArmvelCmd(m_arm, 0.015);
    public ArmvelCmd arm_down = new ArmvelCmd(m_arm, -0.015);
    public HandvelCmd hand_up= new HandvelCmd(m_arm,0.005);
    public HandvelCmd hand_down= new HandvelCmd(m_arm,-0.005);
    public IntakeCmd intakein = new IntakeCmd(m_intake,0.9);
    public IntakeCmd intakeonlyin = new IntakeCmd(m_intake,0.9);
    public IntakeCmd intakestop = new IntakeCmd(m_intake, 0.0);
    public AutoIntakeCmd autointake = new AutoIntakeCmd(m_intake);
    public IntakeCmd m_intakeout = new IntakeCmd(m_intake, -0.5);
    public ShootCmd shoot =  new ShootCmd(m_shooter, m_arm,90.0,ArmConstants.firstarm);//1.2
    public ShootCmd shootb =  new ShootCmd(m_shooter, m_arm,95.0,ArmConstants.secondarm);
    public ShootCmd shootc =  new ShootCmd(m_shooter, m_arm,30.0,ArmConstants.amparm);
    public ShootCmd shootd =  new ShootCmd(m_shooter,m_arm,80.0,ArmConstants.traparm);
    public ShootCmd shoot_sehi = new ShootCmd(m_shooter, m_arm,90.0,ArmConstants.secondhigharm);
    public ShootCmd shoot_thi = new ShootCmd(m_shooter, m_arm,98.0,ArmConstants.thirdarm);
    public ShootCmd shoot_thhi = new ShootCmd(m_shooter, m_arm,80.0,ArmConstants.thirdhigharm);
    public ShootCmd shoot_four = new ShootCmd(m_shooter, m_arm,95.0,ArmConstants.fourtharm);
    public ShootCmd shoot_pass = new ShootCmd(m_shooter, m_arm,60.0,ArmConstants.passarm);
    public OnlyShootCmd only_shoot = new OnlyShootCmd(m_shooter, 10.0);
    public Shoot_standby standby = new Shoot_standby(m_shooter);
    public TriggerCmd trigger = new TriggerCmd(m_shooter, 0.5);
    public TriggerCmd m_triggerout = new TriggerCmd(m_shooter,-0.5);
    public PS5Controller  m_drivestick = new PS5Controller(OIConstants.kDriverControllerPort);
    public PS5Controller  m_drivestick2 = new PS5Controller(OIConstants.kDriver2ControllerPort);
    public DriveCmd drive = new DriveCmd(
                m_robotDrive, 
                () -> -m_drivestick.getRawAxis(Constants.OIConstants.kDriverYAxis), 
                () -> -m_drivestick.getRawAxis(Constants.OIConstants.kDriverXAxis), 
                () ->-m_drivestick.getRawAxis(2), 
                () -> true,
                () -> m_drivestick.getRawButton(12));
            
    // public ArmvelCmd armcontrol = new ArmvelCmd(m_arm, -m_drivestick2.getRawAxis(1)/10);
    // public HandvelCmd handcontrol = new HandvelCmd(m_arm, -m_drivestick2.getRawAxis(5)/10);lic 
    double arm_pos;
    double hand_pos ;
    public Reset reset = new Reset(m_robotDrive);
    public ArmvelCmd arm_control = new ArmvelCmd(m_arm,arm_pos);
    public HandvelCmd hand_control = new HandvelCmd(m_arm,hand_pos);
     public ParallelCommandGroup in = new ParallelCommandGroup(armsuckin,intakein,trigger);
    //  public ParallelCommandGroup in = new ParallelCommandGroup(armsuckin,intakein,trigger);
    // public ParallelCommandGroup autoin = new ParallelCommandGroup(armsuckin,autointake,trigger);
    public ParallelCommandGroup shootfirst = new ParallelCommandGroup(armfirst,shoot);
    public ParallelCommandGroup shootsecond = new ParallelCommandGroup(armsecond,shootb);
    public ParallelCommandGroup shootamp = new ParallelCommandGroup(armamp,shootc);
    public ParallelCommandGroup shoottrap = new ParallelCommandGroup(armtrap,shootd);
    public ParallelCommandGroup shootsecondhigh = new ParallelCommandGroup(armsecondhigh,shoot_sehi);
    public ParallelCommandGroup shootthird = new ParallelCommandGroup(armthird,shoot_thi);
    public ParallelCommandGroup shootthirdhigh = new ParallelCommandGroup(armthirdhigh,shoot_thhi);
    public ParallelCommandGroup shootfourth = new ParallelCommandGroup(armfourth,shoot_four);
    public ParallelCommandGroup shootpass = new ParallelCommandGroup(armpass,shoot_pass);
    public ParallelCommandGroup m_out = new ParallelCommandGroup(m_triggerout,armsuckinspit,m_intakeout);

      //  private final SendableChooser<Command> m_autoChooser;
     
        
    

    
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(){
    // m_autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Mode", m_autoChooser);
    m_robotDrive.setDefaultCommand(
            new DriveCmd(
                m_robotDrive, 
                () ->  -m_drivestick.getRawAxis(Constants.OIConstants.kDriverYAxis), 
                () -> m_drivestick.getRawAxis(Constants.OIConstants.kDriverXAxis),
                () ->m_drivestick.getRawAxis(2)*1.5, 
                () -> true,
                () -> m_drivestick.getRawButton(12)
            )
        );
    //m_shooter.setDefaultCommand(standby);
    oneconfigureButtonBindings();
    twoconfigureButtonBindings();
  
  }
  

  private void oneconfigureButtonBindings() {
     // new JoystickButton(m_drivestick, 7).whileTrue(in);
     // new JoystickButton(m_drivestick, 6).onTrue(armdown);
     // new JoystickButton(m_drivestick, 4).whileTrue(arm_up);
     // new JoystickButton(m_drivestick, 2).whileTrue(arm_down);
     // new JoystickButton(m_drivestick, 1).whileTrue(hand_down);
     // new JoystickButton(m_drivestick, 3).whileTrue(hand_up);
     // new JoystickButton(m_drivestick, 8).onTrue(shootsecond);
    
     // new JoystickButton(m_drivestick, 8).onTrue(shootamp);

     // new JoystickButton(m_drivestick, 8).onTrue(armdown);
     // new JoystickButton(m_drivestick, 6).whileTrue(only_shoot);
     //  new JoystickButton(m_drivestick, 8).whileTrue(reset_dri);
     // new JoystickButton(m_drivestick2, 7).whileTrue(in);
     // new JoystickButton(m_drivestick2, 5).whileTrue(out);

     // new JoystickButton(m_drivestick2, 2).onTrue(shoottrap);
     // new JoystickButton(m_drivestick2, 1).onTrue(shootamp);
     // // new JoystickButton(m_drivestick2, 4).onTrue(armsecond);
     // new JoystickButton(m_drivestick2, 3).onTrue(armdown);
     // new JoystickButton(m_drivestick2, 6).whileTrue(shootsecond);
     // new JoystickButton(m_drivestick2, 8).whileTrue(shootfirst);

     // NamedCommands.registerCommand("Shoot",m_shooter.shootCommand(0.5));
     // NamedCommands.registerCommand("Take",m_shooter.takeCommand());
     // NamedCommands.registerCommand("Intake",m_intake.intakeCommand());
     // NamedCommands.registerCommand("armspeaker",m_Arm.speakershoot());
     new POVButton(m_drivestick, 0).whileTrue(reset);
     // new JoystickButton(m_drivestick, 6).onTrue(armdown);
     new JoystickButton(m_drivestick, 4).whileTrue(arm_up);
     new JoystickButton(m_drivestick, 2).whileTrue(arm_down);
     new JoystickButton(m_drivestick, 1).whileTrue(hand_down);
     new JoystickButton(m_drivestick, 3).whileTrue(hand_up);
     new JoystickButton(m_drivestick, 6).whileTrue(only_shoot);//new
     //new JoystickButton(m_drivestick, 5).onTrue(armclimb);
     //new JoystickButton(m_drivestick, 7).whileTrue(arm_down);
     new JoystickButton(m_drivestick,8).whileTrue(in);

    
    // new JoystickButton(m_drivestick, 1).whileTrue(hand_down);
    // new JoystickButton(m_drivestick, 3).whileTrue(hand_up);
    NamedCommands.registerCommand("onlyshoot", only_shoot);
    NamedCommands.registerCommand("shootfirst", shootfirst);
    NamedCommands.registerCommand("shootsecond", shootsecond);
    // NamedCommands.registerCommand("intake", in);
    // NamedCommands.registerCommand("intakestop", intakestop);
 

  }
  public void twoconfigureButtonBindings(){
   // new JoystickButton(m_drivestick2, 7).whileTrue(in);

    // if(m_drivestick2.getPOV()==0)shootthirdhigh.schedule();
    // if(m_drivestick2.getPOV()==90)shootthird.schedule();
    // if(m_drivestick2.getPOV()==180)shoottrap.schedule();
    // if(m_drivestick2.getPOV()==270)shootfourth.schedule();
    // new CommandJoystick(0).axisGreaterThan(0,0.05,arm_p);

    // if(m_drivestick2.getRawAxis(5)<=-0.15)arm_up.schedule();
    // if(m_drivestick2.getRawAxis(5)>=0.15)arm_down.schedule();
    // if(m_drivestick2.getRawAxis(1)<=-0.15)hand_up.schedule();
    // if(m_drivestick2.getRawAxis(1)>=0.15)hand_down.schedule();

    
    new JoystickButton(m_drivestick2, 6).onTrue(armdown);
    new JoystickButton(m_drivestick2, 8).whileTrue(shootfirst);
    // new JoystickButton(m_drivestick2, 5).whileTrue(intakeout);

    // new JoystickButton(m_drivestick2, 2).onTrue(shoottrap);
    // new JoystickButton(m_drivestick2, 8).onTrue(shootsecond);
    

    new JoystickButton(m_drivestick2, 4).whileTrue(shootsecondhigh);
    new JoystickButton(m_drivestick2, 3).whileTrue(shootsecond);
    new JoystickButton(m_drivestick2, 1).whileTrue(shootamp);
    new JoystickButton(m_drivestick2, 2).whileTrue(shootpass);
    new POVButton(m_drivestick2, 0).whileTrue(shootthirdhigh);
    new POVButton(m_drivestick2, 90).whileTrue(shootthird);
    new POVButton(m_drivestick2, 180).whileTrue(shoottrap);
    new POVButton(m_drivestick2, 270).whileTrue(shootfourth);
    new JoystickButton(m_drivestick2,5).whileTrue(m_out);
    // new JoystickButton(m_drivestick2, 3).onTrue(armdown);
    // new JoystickButton(m_drivestick2, 6).whileTrue(shootsecond);
    // new JoystickButton(m_drivestick2, 6).whileTrue(shootfirst);
  }

public Command getAutonomousCommand() {
  return new onlymove();
  // return new automiddle().withTimeout(15);
  // return new autoleft().withTimeout(15);
  // return new autoright().withTimeout(15);
  // return new threeautoleft().withTimeout(15);
  // return new threeautoright().withTimeout(15);

}

}