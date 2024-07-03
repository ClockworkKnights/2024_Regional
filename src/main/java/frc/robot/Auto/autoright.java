// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.TriggerCmd;
import frc.robot.Auto.suckin;

import javax.swing.Spring;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoright extends SequentialCommandGroup {
  String Path = "two_right";
  /** Creates a new autonight. */
  public autoright() {
    addRequirements(RobotContainer.m_robotDrive);
    // addRequirements(RobotContainer.m_shooter);
    //  addRequirements(RobotContainer.m_arm);
    //  addRequirements(RobotContainer.m_intake);
    



    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( ()->{
        if(DriverStation.getAlliance().get()==Alliance.Red)
            RobotContainer.m_robotDrive.resetOdometry(PathPlannerPath.fromPathFile(Path).flipPath().getPreviewStartingHolonomicPose());
            else 
            {
                RobotContainer.m_robotDrive.resetOdometry(PathPlannerPath.fromPathFile(Path).getPreviewStartingHolonomicPose());
            }
        }

            ),
            new ShootCmd(RobotContainer.m_shooter, RobotContainer.m_arm, 90.0,ArmConstants.firstarm).withTimeout(3.5),
            RobotContainer.m_robotDrive.followPathCommand(Path).raceWith(new suckin()),
             new ShootCmd(RobotContainer.m_shooter, RobotContainer.m_arm, 90.0,ArmConstants.firstarm).withTimeout(3.5)
        //   if(DriverStation.getAlliance().get()==Alliance.Red)
        //   RobotContainer.m_robotDrive.ResetOdometry(PathPlannerPath.fromPathFile(""))
        // })
    );

  }
}
