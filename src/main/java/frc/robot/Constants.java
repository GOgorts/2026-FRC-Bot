// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  
  public static final class IntakeSubsystemConstants{
    
    public static final class IntakeSetpoints {
      public static final double kForward = 0.7;
      public static final double kReverse = -0.7;
      public static final double kHold = 0.25;
    }

  
      public static final class flipSetpoints {
      //public static final double kStow = -5;
      public static final double kUp = -5; //-7 orig
      public static final double kDown = 0; //-20 orig
      public static final double kIn = 0.8;
    }

  }

  public static final class ShooterSubsystemConstants{

      public static final class TurningSetpoints {
        public static final double kturnForawrd = 0.9;
        public static final double ktunReverse = -0.9;
      }
      public static final class TurretSetpoints {
        public static final double kPos = 0.2;
        public static final double kNeg = -0.2;
      }

      public static final class TurretTracking {
        // Proportional gain: output power per degree of TX error (vision tracking)
        public static final double kP = 0.02;
        // Proportional gain: output power per degree of angle error (pose tracking)
        public static final double kPoseP = SmartDashboard.getNumber("Pose Tracking Gain", 0.02);
        // Turret stops correcting when error is within this many degrees
        public static final double kDeadband = 2.0;
        // Maximum output power allowed during tracking
        public static final double kMaxPower = 0.5;
        // Low-pass filter weight for incoming TX (0 = frozen, 1 = no filtering)
        public static final double kTXFilterAlpha = 1;
        // Seconds to continue tracking on last known TX after losing the target
        public static final double kTargetLostTimeoutSecs = 0.3;

        // Encoder rotations per one full turret rotation (gear ratio).
        // To measure: command the turret to rotate exactly 360° and read totalRot.
        public static final double kEncoderToTurretRatio = 10.0;

        // Hub center positions in WPILib field coordinates (origin = blue alliance corner).
        // X = distance from blue alliance wall, Y = distance from bottom field border.
        // Measured from CAD: 181.928" from blue wall, 158.844" from bottom border.
        public static final Translation2d kBlueHubCenter = new Translation2d(
            Units.inchesToMeters(181.928), Units.inchesToMeters(158.844));
        public static final Translation2d kRedHubCenter = new Translation2d(
            Units.inchesToMeters(651.25 - 181.928), Units.inchesToMeters(158.844));
      }

    }
  public static final class HangSubsystemConstants {
    // public static final int kHangMotorCanId = 13;

    public static final double kForward = 0.75;
    public static final double kReverse = -0.7;
    //public static final double kHold = 0.3;

  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    // public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxSpeedMetersPerSecond = 2.5;
    public static final double kMaxAngularSpeed = 2.5 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 8;
    

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
    public static final double kTriggerButtonThreshold = 0.2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VisionConstants {
    /** NetworkTables name of the Limelight (matches the hostname set in the Limelight web UI). */
    public static final String kLimelightName = "limelight-turret";

    /** Pipeline index used on startup. */
    public static final int kDefaultPipeline = 0;

    /**
     * Maximum average tag distance (meters) below which a pose estimate is
     * considered trustworthy enough to pass to the pose estimator.
     */
    public static final double kMaxTrustableTagDistance = 4.0;

    /**
     * Standard deviations for vision pose measurements fed to
     * SwerveDrivePoseEstimator [x (m), y (m), theta (rad)].
     * Increase these to trust vision less relative to wheel odometry.
     */
    public static final double kVisionStdDevX = 0.5;
    public static final double kVisionStdDevY = 0.5;
    public static final double kVisionStdDevTheta = 999999; // ignore heading from vision
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.922; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads =
        Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
    public static final double kIntakeShortBarLength = 0.1524;
    public static final double kIntakeLongBarLength = 0.3048;
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  }
}
