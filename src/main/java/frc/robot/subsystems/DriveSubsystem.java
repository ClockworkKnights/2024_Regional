// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;

// import com.pathplanner.lib.auto.AutoBuilder.*;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriverConstants.kFrontLeftDrivingCanId,
      DriverConstants.kFrontLeftTurningCanId,
      DriverConstants.kFrontLeftChassisAngularOffset,
      DriverConstants.kFrontLeftCanCoderId,
      DriverConstants.kFrontLeftDrivingInvert,
      DriverConstants.kFrontLeftTurningInvert);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriverConstants.kFrontRightDrivingCanId,
      DriverConstants.kFrontRightTurningCanId,
      DriverConstants.kFrontRightChassisAngularOffset,
      DriverConstants.kFrontRightCanCoderId,
      DriverConstants.kFrontRightDrivingInvert,
      DriverConstants.kFrontRightTurningInvert);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriverConstants.kRearLeftDrivingCanId,
      DriverConstants.kRearLeftTurningCanId,
      DriverConstants.kBackLeftChassisAngularOffset,
      DriverConstants.kRearLeftCanCoderId,
      DriverConstants.kRearLeftDrivingInvert,
      DriverConstants.kRearLeftTurningInvert);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriverConstants.kRearRightDrivingCanId,
      DriverConstants.kRearRightTurningCanId,
      DriverConstants.kBackRightChassisAngularOffset,
      DriverConstants.kRearRightCanCoderId,
      DriverConstants.kRearRightDrivingInvert,
      DriverConstants.kRearRightTurningInvert);

  // The gyro sensor
  public final Pigeon2 m_gyro = new Pigeon2(0);
  // private final DutyCycleOut control = new DutyCycleOut(getHeading());
  private Field2d field = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriverConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriverConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriverConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw().getValue()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    resetEncoders();
    zeroHeading();
  }

  public void DEBUG_printTurnEncoders() {
    System.out.println(
        m_frontLeft.getPosition() + " " +
            m_frontRight.getPosition() + " " +
            m_rearLeft.getPosition() + " " +
            m_rearRight.getPosition());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("0", m_frontLeft.absoluteEncoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("1", m_frontRight.absoluteEncoder.getAbsolutePosition().getValue() * 2 * Math.PI);
    SmartDashboard.putNumber("2", m_rearRight.absoluteEncoder.getAbsolutePosition().getValue() * 2 * Math.PI);
    SmartDashboard.putNumber("3", m_rearLeft.absoluteEncoder.getAbsolutePosition().getValue() * 2 * Math.PI);
    // DEBUG_printTurnEncoders();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  // 来自3015的speed和relativespeed
  /**
   * Drive the robot with field relative chassis speeds
   *
   * @param fieldRelativeSpeeds Field relative chassis speeds
   */
  //
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
    return states;
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return DriverConstants.kDriveKinematics.toChassisSpeeds(getStates());
  }

  // 根据速度设定去运动！
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] targetStates = DriverConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, AutoConstants.kMaxSpeedMetersPerSecond);
    // if (targetStates[0].speedMetersPerSecond <= 0.1)
    // System.out.println("!!!");
    // targetStates[0].angle = m_frontLeft.getPosition().angle;
    // if (targetStates[1].speedMetersPerSecond <= 0.1)
    // targetStates[1].angle = m_frontRight.getPosition().angle;
    // if (targetStates[2].speedMetersPerSecond <= 0.1)
    // targetStates[2].angle = m_rearRight.getPosition().angle;
    // if (targetStates[3].speedMetersPerSecond <= 0.1)
    // targetStates[3].angle = m_rearLeft.getPosition().angle;
    // System.out.println("speed0:" + targetStates[0].speedMetersPerSecond);
    // System.out.println("speed1:" + targetStates[1].speedMetersPerSecond);
    // System.out.println("speed2:" + targetStates[2].speedMetersPerSecond);
    // System.out.println("speed3:" + targetStates[3].speedMetersPerSecond);
    // System.out.println(targetStates[0].angle.getDegrees());
    // System.out.println(targetStates[1].angle.getDegrees());
    // System.out.println(targetStates[2].angle.getDegrees());
    // System.out.println(targetStates[3].angle.getDegrees());
    setModuleStates(targetStates);
  }

  // public void getswerveangle(ChassisSpeeds speeds){
  // SwerveModuleState[] targetStates =
  // DriverConstants.kDriveKinematics.toSwerveModuleStates(speeds);
  // SmartDashboard.putNumber("angle0",targetStates[0].angle.getDegrees());
  // SmartDashboard.putNumber("angle1",targetStates[1].angle.getDegrees());// }
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond,
        fieldRelativeSpeeds.omegaRadiansPerSecond,
        Rotation2d.fromDegrees(m_gyro.getAngle())
    // getPose().getRotation()
    );

    driveRobotRelative(speeds);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed); // 计算xy夹角弧度
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));// 计算斜边

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriverConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;

      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

      if (angleDif < 0.025 * Math.PI) {

        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.875 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriverConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriverConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriverConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriverConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriverConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

  }

  // public ChassisSpeeds getRobotRelativeSpeeds(){
  // return ChassisSpeeds.fromFieldRelativeSpeeds(getRobotRelativeSpeeds(), null)
  // }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));// -45
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));// -45
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriverConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);

  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();

  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriverConstants.kGyroReversed ? -1.0 : 1.0);
  }
  // public Command rotaCommand()
  // {
  // return this.runOnce(()->drive(0,0,180,true,false));
  // }

  public void stopModules() {
    m_frontLeft.stopMotor();
    m_frontRight.stopMotor();
    m_rearLeft.stopMotor();
    m_rearRight.stopMotor();
  }

  // generate a trajectory(from .path to .json)
  public PathPlannerPath generatePath(String pathName) {
    return PathPlannerPath.fromPathFile(pathName);
  }

  // return an auto command
  public Command followPathCommand(String Pathname) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(Pathname);
    // You must wrap the path following command in a FollowPathWithEvents command in
    // order for event markers to work
    return new FollowPathHolonomic(
        path,
        this::getPose, // Robot pose supplier
        this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(3.0, 0.0, 0.1), // Translation PID constants
            new PIDConstants(3.0, 0.0, 0.1), // Rotation PID constants
            AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            0.40299, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public DriverStation.Alliance getAlliance() {
    try {
      return DriverStation.getAlliance().get();
    } catch (Exception e) {
      return DriverStation.Alliance.Red;
    }
  }

  public Pose2d inversePose2dUsingAlliance(Pose2d pose, DriverStation.Alliance allianceColor) {
    if (allianceColor != getAlliance()) {
      return new Pose2d(16.54 - pose.getX(), pose.getY(), new Rotation2d(Math.PI).minus(pose.getRotation()));
    }
    return pose;
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValue());
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = m_frontLeft.getPosition();
    positions[1] = m_frontRight.getPosition();
    positions[2] = m_rearLeft.getPosition();
    positions[3] = m_rearRight.getPosition();

    return positions;
  }

  public void setPose(Pose2d pose) {
    m_odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);

  }

  // public void setPose(Pose2d pose) {
  // m_odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  // }
}
