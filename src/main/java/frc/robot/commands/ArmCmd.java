// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmCmd extends Command {
  /** Creates a new ArmCmd. */
  private ArmSubsystem ArmSubsystem; 
  double armposition;
  double handposition;
  Boolean invert;
  public ArmCmd(ArmSubsystem ArmSubsystem , double armposition, double handposition) {
        this.ArmSubsystem = ArmSubsystem;
        this.armposition = armposition;
        this.handposition = handposition;
        addRequirements(ArmSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // if(ArmSubsystem.getArmCANPosition()>=7.392253416820905) 
      ArmSubsystem.setarmposition(armposition);
    // else ArmSubsystem.stopMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      ArmSubsystem.stopMotor();
      //  ArmSubsystem.setposition(armposition, handposition);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
