// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.studica.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AllianceHelper;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DiagnosticConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ModuleConstants;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kUSB1);

  // Pose estimator fuses wheel odometry + Limelight vision measurements
  private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-1 * m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition() },
      new Pose2d(),
      // State std devs: trust wheel odometry tightly (x, y, heading)
      VecBuilder.fill(0.05, 0.05, Math.toRadians(5)),
      // Vision std devs: trust Limelight loosely — tightened dynamically in
      // periodic()
      VecBuilder.fill(0.5, 0.5, Math.toRadians(30)));

  private VisionSubsystem m_vision;

  private final Field2d m_field = new Field2d();

  // Setpoint generator for PathPlanner
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(VisionSubsystem vision) {
    m_vision = vision;
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("Drive/Gyro", m_gyro);

    // Publish diagnostic defaults so they appear as editable entries in Elastic
    SmartDashboard.putNumber(DiagnosticConstants.kWheelRpmKey, DiagnosticConstants.kDefaultDiagnosticWheelRpm);
    SmartDashboard.putString("Diagnostic/ActiveWheel", "None");
    SmartDashboard.putNumber("Diagnostic/TargetRPM", 0);
    SmartDashboard.putNumber("Diagnostic/TargetSpeedMs", 0);

    // Get the robot configuration from PathPlanner's GUI settings
    RobotConfig robotConfig = null;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Capture startup heading as zero once NavX has had time to settle.
    new Thread(() -> {
      try {
        int attempts = 0;
        while (m_gyro.isCalibrating() && attempts < 300) {
          Thread.sleep(20);
          attempts++;
        }
        Thread.sleep(100);
        zeroHeadingWithAdjustment();
      } catch (Exception e) {
      }
    }).start();

    /*
     * Auto Builder & Path Planner Configuration
     */
    AutoBuilder.configure(this::getPose, this::resetOdometry, this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> drive(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(AutoConstants.kPXController, 0, 0),
            new PIDConstants(AutoConstants.kPThetaController, 0, 0)),
        robotConfig, () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
            // return false;
          }
          return false;
          // return false;
        },
        this);

    // Set up for being able to drive using robot-relative ChassisSpeeds
    setpointGenerator = new SwerveSetpointGenerator(
        robotConfig, // The robot configuration. This is the same config used for generating
                     // trajectories and running path following commands.
        DriveConstants.kMaxAngularSpeed // The max rotation velocity of a swerve module in radians per second.
    );

    // Initialize the previous setpoint to the robot's current speeds & module
    // states
    ChassisSpeeds currentSpeeds = getRobotRelativeSpeeds(); // Method to get current robot-relative chassis speeds
    SwerveModuleState[] currentStates = getCurrentModuleStates(); // Method to get the current swerve module states
    previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates,
        DriveFeedforwards.zeros(robotConfig.numModules));
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(-1 * m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition() });

    m_field.setRobotPose(getPose());

    Pose2d pose = getPose();
    SmartDashboard.putNumber("Drive/HeadingDeg", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Drive/PoseX", pose.getX());
    SmartDashboard.putNumber("Drive/PoseY", pose.getY());
    SmartDashboard.putNumber("Drive/RawGyroDeg", -1 * m_gyro.getAngle());
    SmartDashboard.putBoolean("Drive/GyroConnected", m_gyro.isConnected());
    SmartDashboard.putBoolean("Drive/GyroCalibrating", m_gyro.isCalibrating());

    // Send the raw gyro heading (not the fused estimate) to the Limelight so
    // MegaTag2 uses a heading that cannot be corrupted by bad vision measurements.
    if (m_vision != null) {
      m_vision.setRobotOrientation(getHeading(), getTurnRate());
    }

    // Feed Limelight (MT2) pose into the estimator when the fix looks reliable
    if (m_vision != null && m_vision.hasTarget()) {
      double tagDistance = m_vision.getAverageTagDistance();
      int tagCount = m_vision.getTagCount();

      // Only accept fixes within 4 meters; require 2+ tags beyond 2 meters
      if (tagDistance < 4.0 && (tagCount >= 2 || tagDistance < 2.0)) {
        Pose2d visionPose = m_vision.getRobotPose();

        // Reject implausible jumps: if the proposed pose is more than 1 m from
        // the current estimate the measurement is likely stale or corrupt.
        double jumpMeters = visionPose.getTranslation()
            .getDistance(getPose().getTranslation());
        if (jumpMeters > 1.0) {
          return;
        }

        // The camera sits on a rotating turret, so its reported position
        // shifts as the turret rotates (the fixed camera-to-robot offset
        // in the Limelight settings becomes wrong). Use loose std devs
        // so wheel odometry dominates position; vision just corrects
        // long-term drift. Heading is fully ignored (turret yaw ≠ robot yaw).
        double xyStdDev = 0.5 * tagDistance;
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE);

        m_odometry.addVisionMeasurement(
            visionPose,
            m_vision.getPoseTimestamp(),
            visionStdDevs);
      }
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /*public Pose2d setPose() {

    return m_odometry.resetPose([]);
  }*/

  /**
   * Returns the current robot-relative ChassisSpeeds
   *
   * @return The Chassis speeds.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-1 * m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(-1 * m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * This method will take in desired robot-relative chassis speeds,
   * generate a swerve setpoint, then set the target state for each module
   *
   * @param speeds The desired robot-relative speeds
   */
  public void drive(ChassisSpeeds speeds) {
    // Note: it is important to not discretize speeds before or after
    // using the setpoint generator, as it will discretize them for you
    previousSetpoint = setpointGenerator.generateSetpoint(
        previousSetpoint, // The previous setpoint
        speeds, // The desired target speeds
        0.02 // The loop time of the robot code, in seconds
    );
    setModuleStates(previousSetpoint.moduleStates()); // Method that will drive the robot given target module states
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return this.run(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
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

  /**
   * Resets odometry to the alliance-specific field localisation point while
   * preserving the current heading.
   */
  public Command resetOdometryToFieldPointCommand() {
    return this.runOnce(() -> {
      var point = AllianceHelper.isRedAlliance()
          ? FieldConstants.kRedLocalisationPoint
          : FieldConstants.kBlueLocalisationPoint;
      Pose2d pose = new Pose2d(point, getPose().getRotation());
      resetOdometry(pose);
    });
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeadingCommand() {
    return this.runOnce(() -> {
      if (m_gyro.isCalibrating()) {
        DriverStation.reportWarning("NavX reports calibrating; using software yaw zero", false);
      }
      zeroHeadingWithAdjustment();
    });
  }

  /** Sets the angle to offset the robot */
  public Command setAngleOffsetCommand(double offset) {
    return this.runOnce(() -> m_gyro.setAngleAdjustment(offset));
  }

  private void zeroHeadingWithAdjustment() {
    // Software yaw zero: avoids firmware zeroYaw() rejection when NavX reports
    // calibration.
    m_gyro.setAngleAdjustment(0.0);
    m_gyro.setAngleAdjustment(-m_gyro.getAngle());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-1 * m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -1 * m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the current Swerve Module States of the robot
   *
   * @return The Swerve Module States of the robot
   */
  public SwerveModuleState[] getCurrentModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    return states;
  }

  // =========================================================
  // DIAGNOSTIC MODE — per-wheel spin at configurable RPM
  // =========================================================

  /**
   * Converts a drive-motor RPM to the meters/second setpoint expected by
   * {@link MAXSwerveModule#setDesiredState}. The encoder velocity conversion
   * factor already bakes in gear reduction and wheel circumference, so the
   * closed-loop setpoint is in m/s rather than raw RPM.
   *
   * <p>
   * Formula: v = (rpm / 60) * wheelCircumference / gearReduction
   */
  private static double rpmToMetersPerSecond(double rpm) {
    return (rpm / 60.0) * ModuleConstants.kWheelCircumferenceMeters / ModuleConstants.kDrivingMotorReduction;
  }

  /**
   * Spins a single drive wheel forward at the RPM currently set on
   * SmartDashboard ("Diagnostic/WheelRPM") while holding all other wheels
   * stationary. The wheel angle is locked at 0° (forward).
   *
   * @param moduleIndex 0 = front-left, 1 = front-right, 2 = rear-left, 3 =
   *                    rear-right
   */
  private void runSingleModuleDiagnostic(int moduleIndex) {
    double rpm = SmartDashboard.getNumber(
        DiagnosticConstants.kWheelRpmKey, DiagnosticConstants.kDefaultDiagnosticWheelRpm);
    double speedMs = rpmToMetersPerSecond(rpm);

    SwerveModuleState running = new SwerveModuleState(speedMs, Rotation2d.fromDegrees(0));
    SwerveModuleState stopped = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

    m_frontLeft.setDesiredState(moduleIndex == 0 ? running : stopped);
    m_frontRight.setDesiredState(moduleIndex == 1 ? running : stopped);
    m_rearLeft.setDesiredState(moduleIndex == 2 ? running : stopped);
    m_rearRight.setDesiredState(moduleIndex == 3 ? running : stopped);

    SmartDashboard.putString("Diagnostic/ActiveWheel",
        new String[] { "FrontLeft", "FrontRight", "RearLeft", "RearRight" }[moduleIndex]);
    SmartDashboard.putNumber("Diagnostic/TargetRPM", rpm);
    SmartDashboard.putNumber("Diagnostic/TargetSpeedMs", speedMs);
  }

  /** Stops all wheels and clears the active-wheel diagnostic indicator. */
  private void stopAllModulesDiagnostic() {
    SwerveModuleState stopped = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    m_frontLeft.setDesiredState(stopped);
    m_frontRight.setDesiredState(stopped);
    m_rearLeft.setDesiredState(stopped);
    m_rearRight.setDesiredState(stopped);
    SmartDashboard.putString("Diagnostic/ActiveWheel", "None");
  }

  /**
   * Returns a command that runs the front-left wheel at the diagnostic RPM
   * (toggle on/off with the same button via {@code toggleOnTrue}).
   */
  public Command diagFrontLeftCommand() {
    return this.startEnd(() -> runSingleModuleDiagnostic(0), this::stopAllModulesDiagnostic)
        .withName("DiagFrontLeft");
  }

  /** @see #diagFrontLeftCommand() */
  public Command diagFrontRightCommand() {
    return this.startEnd(() -> runSingleModuleDiagnostic(1), this::stopAllModulesDiagnostic)
        .withName("DiagFrontRight");
  }

  /** @see #diagFrontLeftCommand() */
  public Command diagRearLeftCommand() {
    return this.startEnd(() -> runSingleModuleDiagnostic(2), this::stopAllModulesDiagnostic)
        .withName("DiagRearLeft");
  }

  /** @see #diagFrontLeftCommand() */
  public Command diagRearRightCommand() {
    return this.startEnd(() -> runSingleModuleDiagnostic(3), this::stopAllModulesDiagnostic)
        .withName("DiagRearRight");
  }
}
