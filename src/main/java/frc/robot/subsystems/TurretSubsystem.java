package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants.TurretSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants.TurretTracking;

public class TurretSubsystem extends SubsystemBase {

    private static final int turretTurnCanID = 16;
    private static final double MaxRot = 2.5;
    private static final double MinRot = -2.5;

    private SparkMax turretMotor = new SparkMax(turretTurnCanID, MotorType.kBrushless);
    private AbsoluteEncoder turretEncoder;
    private double lastPos = 0;
    private double totalRot = 0;

    private final VisionSubsystem m_vision;
    private double m_filteredTX = 0.0;
    private double m_lastTargetSeenTime = 0.0;

    public TurretSubsystem(VisionSubsystem vision) {
        m_vision = vision;
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
        SmartDashboard.putBoolean("Turret/HasTarget", m_vision.hasTarget());
        SmartDashboard.putNumber("Turret/TargetTX", m_vision.getTX());
        SmartDashboard.putNumber("Turret/FilteredTX", m_filteredTX);
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
