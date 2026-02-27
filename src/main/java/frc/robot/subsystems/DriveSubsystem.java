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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCan     DriveConstants.k
      DriveConstants.kFrontLeftChassisAngula
      
      final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCan     DriveConstants.k
      DriveConstants.kFrontRightChassisAngula
      
      final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCan     DriveConstants.k
      DriveConstants.kBackLeftChassisAngula
      
      final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCan     DriveConstants.k
      DriveConstants.kBackRightChassisAngula
      
      yro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // Pose estimator fuses wheel odometry + Limelight vision measurements
  private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-1 * m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()},
      new Pose2d(),
      // State std devs: trust whee l odometry tightly (x, y, heading)
      VecBuilder.fill(0.05, 0.05, Math.toRadians(5)),
      // Vision std devs: trust Limelight loosely — tightened dynamically in periodic()
      VecBuilder.fill(0.5, 0.5, Math.toRadians(30)));

      // 
  private VisionSubsystem m_vision;

  // Setpoint generator for PathPlanner
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(VisionSubsystem vision) {
    m_vision = vision;

    // Get the robot configuration from PathPlanner's GUI settings
    RobotConfig robotConfig = null;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Zero the yaw on a separate thread after waiting a second for the NavX2 to calibrate.
    // zeroYaw() is used instead of reset() because reset() triggers a full sensor
    // recalibration on the NavX2, which takes several seconds and disrupts odometry.
    new Thread(() -> {
      try {
        Thread.sleep(1000);
    // 
        m_gyro.zeroYaw();
      } catch (Exception e) {}
    // 
    }).start();
    
    /* 
     *    Auto Builder & Path Planner Configuration
     */
      
    AutoBuilder

      
       PIDConstants(AutoConstants.kPXController, 0, 0), 
      new PIDConstants(AutoConstants.kPThetaController, 0, 0)), 
      robotConfig,  () -> {
          // Boolean supplier that controls when
          // This will flip the path be
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            
          var allian  = Driv
          if (alliance.isPresent()) {
          // 
            return alliance.get() == DriverStation.Alliance.Red;
            // return false;

            return false;
            // return false;
            
            );
          
    // Set up for being able to drive using robot-relative ChassisSpeeds
    setpointGenerator = ne
        ro
        DriveConstants.kMaxAngularSpeed // The max rotation velocity of a swerve module in radians per second.
      );

    // Initialize the previous setpoint to the robot
        assisSpeeds currentSpeeds = getRobotRelativeSpeeds(); // Method to get current robot
                     // relative chassis speeds
        erveModuleState[] currentStates = getCurrentModuleStates(); // Method to get the current swerve module
    eviousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(robotConfig.numModules));
  }

    // 
  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(-1 * m_gyro.getAngle()),
        
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()});

            d Limelight pose into the 
            vision != null && m_vision.
            le tagDistance = m_vision
            tagCount = m_vision.getTa gCount();

      // Only accept fixes within 4 meters; require 2+ tags beyond 2 meters
      if (tagDistance < 4.0 && (tagCount >= 2 || tagDistance < 2.0)) {
        // Scale trust inversely with distance: tighter std devs when close
        double xyStdDev = 0.1 * tagDistance;
        double headingStdDev = Math.toRadians(10 * tagDistance);
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xyStdDev, xyStdDev, headingStdDev);

        m_odometry.addVisionMeasurement(
            m_vision.getRobotPose(),
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

  /**
   * Returns the current robot-relative ChassisSpeed 
   *
   * @return The Chassis sp ds.
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
        ethod to drive the robo
        
        param xSpeed Speed of 
        param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinemat
              fieldRelative
                  ? ChassisSpeeds.fromF
                      xSpeedDelivered
                      ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(-1 * m_gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.set        esiredState(swerveModuleStates[2]);
    m_rearRight.se        DesiredState(swerveModuleStates[3]);
  }           

   *                      
  /**
    * This method will take in desired robot-relative chassis speeds,
    * generate a swerve setpoint, then set the target state for each module
    *
    * @param speeds The desired robot-relative speeds
    */
  public void drive(ChassisSpeeds speeds) {
    // Note: it is important sing the setpoint generator, as it will discretize th
        Setpoint = se
            tpoint, // The previous setpoint
                 desired target 
                op time of the r
                
                previousSetpoint.moduleStates()); // Method tha
            

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return this.run(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
     
       });
   
   
   *
    Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredS
        erveDriveKinematics.desaturateWheelSpeeds(
          desiredStates, DriveConstants.kMax
        frontLeft.setDesiredState(desiredStates[0]);
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
  public Command zeroHeadingCommand() {
    return this.runOnce(() -> m_gyro.zeroYaw());
  } 
 
   * 
  /** Sets the angle to offset the robot */
  public Command setAngleOffsetCommand(double offset) {
    return this.runOnce(() -> m_gyro.setAngleAdjustment(offset));
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
   

  public SwerveModuleState [] getCurrentModuleStates() {
    SwerveModuleState [] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    return states;
  }
}
