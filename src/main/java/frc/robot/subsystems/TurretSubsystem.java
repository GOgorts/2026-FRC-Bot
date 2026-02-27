package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants.TurretSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants.TurretTracking;

public class TurretSubsystem extends SubsystemBase {

    private static final int turretTurnCanID = 16;
    private static final double MaxRot = 2.0;
    private static final double MinRot = -2.0;

    private SparkMax turretMotor = new SparkMax(turretTurnCanID, MotorType.kBrushless);
    private AbsoluteEncoder turretEncoder;
    private double lastPos = 0;
    private double totalRot = 0;

    private final VisionSubsystem m_vision;
    private final DriveSubsystem m_drive;
    private double m_filteredTX = 0.0;
    private double m_lastTargetSeenTime = 0.0;

    public TurretSubsystem(VisionSubsystem vision, DriveSubsystem drive) {
        m_vision = vision;
        m_drive = drive;
        turretEncoder = turretMotor.getAbsoluteEncoder();

        turretMotor.configure(
                Configs.ShooterSubsystem.TurretConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        lastPos = turretEncoder.getPosition();
    }

    @Override
    public void periodic() {
        updateRotation();
        SmartDashboard.putNumber("Turret/TotalRot", totalRot);
        SmartDashboard.putNumber("Turret/AngleDeg", getTurretAngleDeg());
        SmartDashboard.putBoolean("Turret/HasTarget", m_vision.hasTarget());
        SmartDashboard.putNumber("Turret/TargetTX", m_vision.getTX());
        SmartDashboard.putNumber("Turret/FilteredTX", m_filteredTX);
    }

    /**
     * Returns the turret's current angle in degrees relative to the robot's
     * forward direction (0° = straight ahead, positive = clockwise when viewed
     * from above). Requires {@code kEncoderToTurretRatio} to be calibrated.
     */
    public double getTurretAngleDeg() {
        return (totalRot / TurretTracking.kEncoderToTurretRatio) * 360.0;
    }

    /**
     * Updates totalRot by accumulating delta from the absolute encoder, handling
     * 0/1 wraparound.
     */
    private void updateRotation() {
        double currentPos = turretEncoder.getPosition(); // 0.0 to 1.0
        double delta = currentPos - lastPos;

        // Correct for wraparound crossing the 0/1 boundary
        if (delta > 0.5) {
            delta -= 1.0;
        } else if (delta < -0.5) {
            delta += 1.0;
        }

        totalRot += delta;
        lastPos = currentPos;
    }

    public double getRotation() {
        return totalRot;
    }

    private void setTurnPower(double power) {
        if (totalRot >= MaxRot && power > 0) return;
        if (totalRot <= MinRot && power < 0) return;
        turretMotor.set(power);
    }

    /**
     * Proportionally drives the turret toward the Limelight's active target.
     *
     * <p>Two stability techniques are applied:
     * <ul>
     *   <li>Exponential moving average smooths noisy TX readings.</li>
     *   <li>A short timeout keeps the turret moving on the last known TX
     *       when the target briefly flickers out, preventing start/stop twitching.</li>
     * </ul>
     */
    private void moveToTarget() {
        double now = Timer.getFPGATimestamp();

        if (m_vision.hasTarget()) {
            m_filteredTX = TurretTracking.kTXFilterAlpha * m_vision.getTX()
                    + (1.0 - TurretTracking.kTXFilterAlpha) * m_filteredTX;
            m_lastTargetSeenTime = now;
        } else if (now - m_lastTargetSeenTime > TurretTracking.kTargetLostTimeoutSecs) {
            m_filteredTX = 0.0;
            turretMotor.set(0.0);
            return;
        }

        if (Math.abs(m_filteredTX) < TurretTracking.kDeadband) {
            turretMotor.set(0.0);
            return;
        }

        double power = MathUtil.clamp(
                -m_filteredTX * TurretTracking.kP,
                -TurretTracking.kMaxPower,
                TurretTracking.kMaxPower);

        setTurnPower(power);
    }

    /**
     * Returns a command that continuously tracks the active Limelight target,
     * stopping the turret motor when the command ends.
     */
    public Command trackTargetCommand() {
        return this.run(this::moveToTarget)
                .finallyDo(() -> turretMotor.set(0.0))
                .withName("TrackTarget");
    }

    /**
     * Drives the turret toward the active alliance's hub using the robot's
     * field-relative pose from odometry.
     *
     * <p>Steps:
     * <ol>
     *   <li>Compute the field-relative angle from the robot to the hub.</li>
     *   <li>Subtract the robot's heading to get the turret-relative target angle.</li>
     *   <li>Compare to the turret's current angle and drive proportionally.</li>
     * </ol>
     */
    private void poseTrack() {
        Pose2d robotPose = m_drive.getPose();

        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation2d hubPos = isRed ? TurretTracking.kRedHubCenter : TurretTracking.kBlueHubCenter;

        double dx = hubPos.getX() - robotPose.getX();
        double dy = hubPos.getY() - robotPose.getY();
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        double desiredAngleDeg = MathUtil.inputModulus(fieldAngleDeg - robotHeadingDeg, -180, 180);

        double error = MathUtil.inputModulus(desiredAngleDeg - getTurretAngleDeg(), -180, 180);

        SmartDashboard.putNumber("Turret/PoseDesiredDeg", desiredAngleDeg);
        SmartDashboard.putNumber("Turret/PoseErrorDeg", error);

        if (Math.abs(error) < TurretTracking.kDeadband) {
            turretMotor.set(0.0);
            return;
        }

        double power = MathUtil.clamp(
            error * TurretTracking.kPoseP,
            -TurretTracking.kMaxPower,
            TurretTracking.kMaxPower);

        setTurnPower(power);
    }

    /**
     * Returns a command that continuously aims the turret at the alliance hub
     * using the robot's odometry pose. Stops the motor when interrupted.
     */
    public Command poseTrackCommand() {
        return this.run(this::poseTrack)
            .finallyDo(() -> turretMotor.set(0.0))
            .withName("PoseTrack");
    }

    public Command posTurretCommand() {
        return this.startEnd(
                () -> this.setTurnPower(TurretSetpoints.kPos),
                () -> this.setTurnPower(0.0));
    }

    public Command negTurretCommand() {
        return this.startEnd(
                () -> this.setTurnPower(TurretSetpoints.kNeg),
                () -> this.setTurnPower(0.0));
    }
}
