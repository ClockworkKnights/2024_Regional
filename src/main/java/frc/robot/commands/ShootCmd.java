// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import java.util.function.DoubleSupplier;

public class ShootCmd extends Command {
  private ShootSubsystem ShootSubsystem; 
  private ArmSubsystem ArmSubsystem;
  Double speed;
  Double goal;
  /** Creates a new ShootCmd. */
  public ShootCmd(ShootSubsystem ShootSubsystem, ArmSubsystem ArmSubsystem, Double speed, Double goal) {
    this.ShootSubsystem = ShootSubsystem;
    this.speed = speed;
    this.ArmSubsystem = ArmSubsystem;
    this.goal = goal;
      addRequirements(ShootSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ShootSubsystem.getshooter();
    if(Math.abs(ArmSubsystem.getArmCANPosition()-goal)<0.1){
      ShootSubsystem.flywheelControlshoot(speed);}
    else{
      ShootSubsystem.flywheelControl(0);
    }
    // SmartDashboard.putNumber("chazhi",Math.abs(ArmSubsystem.getArmCANPosition()-goal));
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