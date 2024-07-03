package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class DriveCmd extends Command {    
    private DriveSubsystem driveSubsystem;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier fieldRelative;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public DriveCmd(DriveSubsystem driveSubsystem, DoubleSupplier translationSup, DoubleSupplier strafeSup,
     DoubleSupplier rotationSup,BooleanSupplier fieldRelative, BooleanSupplier robotCentricSup) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldRelative = fieldRelative;
        this.robotCentricSup = robotCentricSup;
        this.xLimiter = new SlewRateLimiter(Constants.DriverConstants.kMagnitudeSlewRate);
        this.yLimiter = new SlewRateLimiter(Constants.DriverConstants.kMagnitudeSlewRate);
        this.turningLimiter = new SlewRateLimiter(DriverConstants.kRotationalSlewRate);
    }


    @Override
    public void initialize() {
        super.initialize();
    }

    public static double speedX;
    public static double speedY;
    @Override
    public void execute() {
        // SmartDashboard.putNumber("x",)
        // 1. Get real-time joystick inputs
        double xSpeed = translationSup.getAsDouble();
        double ySpeed = strafeSup.getAsDouble();
        double turningSpeed = rotationSup.getAsDouble();

        speedX=xSpeed;
        speedY=ySpeed;
        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > 0.05 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > 0.05 ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriverConstants.kMagnitudeSlewRate;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriverConstants.kMagnitudeSlewRate;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriverConstants.kRotationalSlewRate;
        
        //driveSubsystem.m_gyro.getPitch();

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        // SmartDashboard.putNumber("-m_stick1.getRawAxis(OIConstants.kDriverYAxis)", xSpdFunction.get());
        // SmartDashboard.putNumber("m_stick1.getRawAxis(OIConstants.kDriverXAxis)", ySpdFunction.get());
        //SmartDashboard.putNumber("m_stick1.getRawAxis(OIConstants.kDriverRotAxis)", turningSpdFunction.get());
        // SmartDashboard.putBoolean("!m_stick1.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)", fieldOrientedFunction.get());  
        // SmartDashboard.putBoolean("isHeadLess", fieldOrientedFunction.get());
        if (fieldRelative.getAsBoolean()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, driveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);//chassisspeeds
        }

        //SmartDashboard.putString("D1 chassisSpeeds", chassisSpeeds.toString());

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriverConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        driveSubsystem.setModuleStates(moduleStates);
        //SmartDashboard.putString("getRotation2d()", driveSubsystem.getRotation2d().toString());
        // driveSubsystem.drive(
        //     xSpeed, ySpeed, turningSpeed, fieldRelative.getAsBoolean(), robotCentricSup.getAsBoolean());
        
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}