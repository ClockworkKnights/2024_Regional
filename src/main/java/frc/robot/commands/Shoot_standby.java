// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import java.util.function.DoubleSupplier;
public class Shoot_standby extends Command {
  private ShootSubsystem ShootSubsystem; 
  Double speed; 
  /** Creates a new ShootCmd. */
  public Shoot_standby(ShootSubsystem ShootSubsystem) {
    this.ShootSubsystem = ShootSubsystem;
    this.speed = speed;
      addRequirements(ShootSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      ShootSubsystem.standby();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      // ShootSubsystem.setOne();
      ShootSubsystem.flywheelControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
