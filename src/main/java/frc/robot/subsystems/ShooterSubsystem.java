package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants.ShooterSetpoints;

public class ShooterSubsystem extends SubsystemBase {

        private static final int shootingMotorCanID = 15;

        private final SparkMax shootingMotor = new SparkMax(shootingMotorCanID, MotorType.kBrushless);
        private final SparkClosedLoopController shootingController;
        private final RelativeEncoder shootingEncoder;

        private double m_targetRPM = 0;

        public ShooterSubsystem() {
                shootingMotor.configure(
                                Configs.ShooterSubsystem.ShooterConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                shootingController = shootingMotor.getClosedLoopController();
                shootingEncoder = shootingMotor.getEncoder();
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("Shooter/RPM", getVelocityRPM());
                SmartDashboard.putNumber("Shooter/TargetRPM", m_targetRPM);
                SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());
        }

        /** Current shooter wheel speed in RPM. */
        public double getVelocityRPM() {
                return shootingEncoder.getVelocity();
        }

        /** True when the shooter is within tolerance of its target RPM. */
        public boolean isAtSpeed() {
                return m_targetRPM > 0
                        && Math.abs(getVelocityRPM() - m_targetRPM) < ShooterSetpoints.kAtSpeedToleranceRPM;
        }

        private void setShooterRPM(double rpm) {
                m_targetRPM = rpm;
                shootingController.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }

        private void stopShooter() {
                m_targetRPM = 0;
                shootingMotor.set(0.0);
        }

        /** Spins the shooter up to the configured RPM and holds it until interrupted. */
        public Command runShooterCommand() {
                return this.startEnd(
                                () -> this.setShooterRPM(ShooterSetpoints.kForwardRPM),
                                this::stopShooter);
        }

        /** Runs the shooter in reverse at low open-loop power (for clearing jams). */
        public Command reverseShooterCommand() {
                return this.startEnd(
                                () -> shootingMotor.set(ShooterSetpoints.kReversePower),
                                this::stopShooter);
        }
}
