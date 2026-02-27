// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AllianceHelper;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable m_limelightTable;

    public enum LEDMode {
        kPipeline(0), kOff(1), kBlink(2), kOn(3);

        public final int value;

        LEDMode(int value) {
            this.value = value;
        }
    }

    public enum CameraMode {
        kVision(0), kDriver(1);

        public final int value;

        CameraMode(int value) {
            this.value = value;
        }
    }

    public VisionSubsystem() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName);
        setLEDMode(LEDMode.kPipeline);
        setPipeline(VisionConstants.kDefaultPipeline);
    }

    // ---------------------------------------------------------------------------
    // Target data
    // ---------------------------------------------------------------------------

    /**
     * Returns true when the Limelight has a valid target in the current pipeline.
     */
    public boolean hasTarget() {
        return m_limelightTable.getEntry("tv").getDouble(0) == 1.0;
    }

    /** Horizontal offset from crosshair to target (degrees, –29.8° to +29.8°). */
    public double getTX() {
        return m_limelightTable.getEntry("tx").getDouble(0);
    }

    /** Vertical offset from crosshair to target (degrees, –24.85° to +24.85°). */
    public double getTY() {
        return m_limelightTable.getEntry("ty").getDouble(0);
    }

    /** Target area as a percentage of the image (0–100%). */
    public double getTA() {
        return m_limelightTable.getEntry("ta").getDouble(0);
    }

    /** AprilTag ID of the primary in-view target, or –1 if none. */
    public int getTargetID() {
        return (int) m_limelightTable.getEntry("tid").getDouble(-1);
    }

    // ---------------------------------------------------------------------------
    // Pose estimation (MegaTag / AprilTag botpose)
    // ---------------------------------------------------------------------------

    /**
     * Returns the robot's estimated field-relative pose from the Limelight
     * (alliance-origin-corrected). Returns an empty Pose2d if no pose is available.
     */
    public Pose2d getRobotPose() {
        double[] pose = getAllianceBotPoseArray();
        if (pose.length < 6)
            return new Pose2d();
        return new Pose2d(
                new Translation2d(pose[0], pose[1]),
                Rotation2d.fromDegrees(pose[5]));
    }

    /**
     * Returns the FPGA timestamp (seconds) at which the Limelight pose observation
     * was captured. Pass this to SwerveDrivePoseEstimator.addVisionMeasurement().
     */
    public double getPoseTimestamp() {
        double[] pose = getAllianceBotPoseArray();
        if (pose.length < 7)
            return 0.0;
        double latencyMs = pose[6];
        return Timer.getFPGATimestamp() - (latencyMs / 1000.0);
    }

    /**
     * Returns the number of tags used in the current pose estimate.
     * Higher counts mean a more trustworthy estimate.
     */
    public int getTagCount() {
        double[] pose = getAllianceBotPoseArray();
        if (pose.length < 8)
            return 0;
        return (int) pose[7];
    }

    /**
     * Returns the average distance (meters) from the camera to the tags used in
     * the current pose estimate.
     */
    public double getAverageTagDistance() {
        double[] pose = getAllianceBotPoseArray();
        if (pose.length < 10)
            return 0.0;
        return pose[9];
    }

    // ---------------------------------------------------------------------------
    // Robot orientation (MegaTag2)
    // ---------------------------------------------------------------------------

    /**
     * Sends the robot's current heading and yaw rate to the Limelight so it can
     * use MegaTag2 for pose estimation. Must be called every loop before reading
     * botpose_orb values.
     *
     * @param yawDeg            Robot yaw in degrees (WPILib convention: CCW positive)
     * @param yawRateDegPerSec  Robot yaw rate in degrees per second
     */
    public void setRobotOrientation(double yawDeg, double yawRateDegPerSec) {
        m_limelightTable.getEntry("robot_orientation")
                .setDoubleArray(new double[] { yawDeg, yawRateDegPerSec, 0, 0, 0, 0 });
    }

    // ---------------------------------------------------------------------------
    // Camera controls
    // ---------------------------------------------------------------------------

    public void setLEDMode(LEDMode mode) {
        m_limelightTable.getEntry("ledMode").setNumber(mode.value);
    }

    public void setCameraMode(CameraMode mode) {
        m_limelightTable.getEntry("camMode").setNumber(mode.value);
    }

    /** Switches to the specified pipeline index (0–9). */
    public void setPipeline(int pipeline) {
        m_limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    // ---------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------

    private double[] getAllianceBotPoseArray() {
        // Always use blue-origin coordinates so that the pose matches the
        // field positions defined in Constants (hub centers, etc.).
        // Alliance only controls *which* hub to target, not the coordinate frame.
        return m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    }

    // ---------------------------------------------------------------------------
    // Periodic
    // ---------------------------------------------------------------------------

    @Override
    public void periodic() {
        // System.out.println("Vision/TX: " + getTX());
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
        SmartDashboard.putNumber("Vision/TX", getTX());
        SmartDashboard.putNumber("Vision/TY", getTY());
        SmartDashboard.putNumber("Vision/TA", getTA());
        SmartDashboard.putNumber("Vision/TargetID", getTargetID());
        SmartDashboard.putNumber("Vision/TagCount", getTagCount());
        AllianceHelper.publishDiagnostics();
    }
}
