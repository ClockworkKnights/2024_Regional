// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.TriggerCmd;
import frc.robot.subsystems.ShootSubsystem; 
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class suckin extends ParallelCommandGroup {
  /** Creates a new suckin. */
  public suckin() {
    addCommands(new TriggerCmd(RobotContainer.m_shooter, 0.5),
    new ArmCmd(RobotContainer.m_arm, ArmConstants.suckinarm, ArmConstants.suckinhand),
     new IntakeCmd(RobotContainer.m_intake,0.9));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
