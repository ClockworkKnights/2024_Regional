// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.OnlyShootCmd;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class onlyshoot extends SequentialCommandGroup {
  /** Creates a new onlyshoot. */
  public onlyshoot(ShootSubsystem m_shooter, ArmSubsystem m_arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ArmCmd armcmd1 = new ArmCmd(m_arm, ArmConstants.firstarm, ArmConstants.firsthand);
    OnlyShootCmd shootcmd1 = new OnlyShootCmd(m_shooter,90.0);
    addCommands(shootcmd1);//1.2);
  }
}
