// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DriverConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 2.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 3; // 2.5percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.5; // 2percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = 0.55;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.52;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        // new Translation2d(-kWheelBase / 2, kTrackWidth / 2),// 左前
        // new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),// 右前 --
        // new Translation2d(kWheelBase / 2, kTrackWidth / 2),//左后++
        // new Translation2d(kWheelBase / 2, -kTrackWidth / 2));//右后
        // 按照安装
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // 左前
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // 右前
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // 左后
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));// 右后

    // Angular offsets of the modules relative to the cha%ssis in radians

    // public static final double kFrontLeftChassisAngularOffset = 0;//coder0
    // public static final double kFrontRightChassisAngularOffset = 0;//coder1
    // public static final double kBackLeftChassisAngularOffset = 0;//coder3
    // public static final double kBackRightChassisAngularOffset = 0;//coder2

    public static final double kFrontLeftChassisAngularOffset = -3.127786826498822+Math.PI; // coder0;
    public static final double kFrontRightChassisAngularOffset = 0.857495260428073+Math.PI;// coder1
    public static final double kBackLeftChassisAngularOffset = 1.544718653400841;// coder3
    public static final double kBackRightChassisAngularOffset = 1.254796284490455+Math.PI;// coder2

    // // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 16;
    public static final int kFrontRightDrivingCanId = 12;
    public static final int kRearRightDrivingCanId = 14;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 17;
    public static final int kFrontRightTurningCanId = 13;
    public static final int kRearRightTurningCanId = 15;

    // CAN coder ID
    public static final int kFrontLeftCanCoderId = 0;
    public static final int kRearLeftCanCoderId = 3;
    public static final int kFrontRightCanCoderId = 1;
    public static final int kRearRightCanCoderId = 2;

    // reverse

    public static final boolean kFrontLeftDrivingInvert = false;
    public static final boolean kFrontRightDrivingInvert = false;
    public static final boolean kRearLeftDrivingInvert = false;
    public static final boolean kRearRightDrivingInvert = false;

    public static final boolean kFrontLeftTurningInvert = false;
    public static final boolean kFrontRightTurningInvert = false;
    public static final boolean kRearLeftTurningInvert = false;
    public static final boolean kRearRightTurningInvert = false;

    public static final boolean CanCoderInvert = false;

    public static final boolean kGyroReversed = false;

  }
  //  public static final class ArmConstants{
  //    public static final Double suckinarm = 5.471709470388082;
  //    public static final Double suckinhand = 5.488583259054824;

  //  }
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.10;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    //
    public static final double kDrivingMotorReduction = (45.0 * 17 * 50) / (kDrivingMotorPinionTeeth * 15 * 17);//
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kAngleGearRatio = 7 / 150;
    public static final double kTurningEncoderPositionFactor = kAngleGearRatio * (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = kAngleGearRatio * (2 * Math.PI) / 60.0; // radians per
                                                                                                       // second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.005;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }
   public static final class ArmConstants {
    
    public static final Double suckinarm = 2.270291566070749; //7.557301975912328;
     public static final Double suckinhand = 5.583690067903734; //5.516194913236766;

     public static final Double firstarm = 2.624641128072332; //7.610078688700666;
     public static final Double firsthand = 5.962583322511487; //5.654253184146473;
     
     public static final Double secondarm = 2.423689644859313;//7.872389403429111
     public static final Double secondhand = 5.7048745501467;//5.618971626025104
     
    //  public static final Double downarm = 7.258059222759655;
    //  public static final Double downhand = 6.41;
    public static final Double downarm = 2.270291566070749;
     public static final Double downhand = 5.583690067903734;
     
     public static final Double amparm = 3.247437327953902;//8.7!!! 
     public static final Double amphand = 6.100641593421195;//6.0!!!

    //  public static final Double traparm = 5.71868037723767;
    //  public static final Double traphand = 4.489961766141271;//change

    //  public static final Double secondhigharm = 8.7; //8.69522447173604;
    //  public static final Double secondhighhand = 4.78;//4.876524924688454;

    //  public static final Double thirdarm = 7.6;
    //  public static final Double thirdhand = 6.014156171724732;

    //  public static final Double thirdhigharm = 8.746758452523926;
    //  public static final Double thirdhighhand = 4.988505522204105;

    //  public static final Double fourtharm = 7.248059222759655;
    //  public static final Double fourthhand = 6.295457153482672;
    //  public static final Double climbarm = 8.8;
    //  public static final Double climbhand = 6.41;
    
     public static final Double traparm =  2.270291566070749;
     public static final Double traphand = 5.583690067903734;//change

     public static final Double secondhigharm =  2.270291566070749; //8.69522447173604;
     public static final Double secondhighhand = 5.583690067903734;//4.876524924688454;

     public static final Double thirdarm = 2.270291566070749;
     public static final Double thirdhand = 5.583690067903734;

     public static final Double thirdhigharm =  2.270291566070749;
     public static final Double thirdhighhand = 5.583690067903734;

     public static final Double fourtharm =  2.270291566070749;
     public static final Double fourthhand = 5.583690067903734;
     public static final Double climbarm =  2.270291566070749;
     public static final Double climbhand =5.583690067903734;

     public static final Double passarm = 7.784952498519629; //7.909204942338366;
     public static final Double passhand = 6.596117387908257;//5.657321145722245;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(262.86);// Rotation2d.fromDegrees(-285.77539-98.5+360);
    public static final int kMotorPort = 4;

    public static final double kP = 3;
    public static final double kI = 0.0001;
    public static final double kD = 0.001;

    // These are fake gains; in actuality these must be determined individually for
    // each robot
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 18;

    public static final int[] kEncoderPorts = new int[] { 4, 5 };
    public static final int kEncoderPPR = 256;
    public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kArmOffsetRads = 5.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriver2ControllerPort = 1;
    public static final double kDriveDeadband = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // public static final PIDConstants CONSTANTS_X = new PIDConstants(0.1, 0, 0);

    // public static final PIDConstants THETA_CONSTANTS = new PIDConstants(0.01, 0,
    // 0);

  }

  public static final class PathPlannerConstants {
    // public static final PathConstraints constraints = new PathConstraints(2, 1,
    // 0, 0);

  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
