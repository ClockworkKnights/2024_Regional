// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.BooleanSupplier;
public class IntakeCmd extends Command {
  /** Creates a new IntakeCmd. */
  private IntakeSubsystem IntakeSubsystem; 
  Boolean invert;
  double speed;
  public IntakeCmd(IntakeSubsystem IntakeSubsystem, double speed) {
    this.IntakeSubsystem = IntakeSubsystem;
    this.speed = speed;
    addRequirements(IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSubsystem.intake(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.setZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return is_loaded==true;
    return false;
  }
}
