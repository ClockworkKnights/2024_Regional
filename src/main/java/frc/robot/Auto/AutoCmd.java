package frc.robot.Auto;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class AutoCmd extends SequentialCommandGroup {
     public AutoCmd(DriveSubsystem s_Swerve){
        // addCommands(
        // );
        String path="two_middle";
        addCommands(        
             new InstantCommand(() -> s_Swerve.setPose(s_Swerve.generatePath(path).getPreviewStartingHolonomicPose())),
            
            s_Swerve.followPathCommand(path)
        );
    
    }
}
