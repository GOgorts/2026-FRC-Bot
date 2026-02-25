package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants.ShooterSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants.TurretSetpoints;

public class ShooterSubsystem extends SubsystemBase {

        private static final int shootingMotorCanID = 15;
        private static final int turretTurnCanID = 16;
        private static final double MaxRot = 2.5;
        private static final double MinRot = -2.5;

        private SparkMax shootingMotor = new SparkMax(shootingMotorCanID, MotorType.kBrushless);
        private SparkMax turretMotor = new SparkMax(turretTurnCanID, MotorType.kBrushless);
        private AbsoluteEncoder turretEncoder;
        private double lastPos = 0;
        private double totalRot = 0;

        public ShooterSubsystem() {
                turretEncoder = turretMotor.getAbsoluteEncoder();

                shootingMotor.configure(
                                Configs.ShooterSubsystem.ShooterConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                turretMotor.configure(
                                Configs.ShooterSubsystem.TurretConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                lastPos = turretEncoder.getPosition();
        }

        @Override
        public void periodic() {
                updateRotation();
                SmartDashboard.putNumber("Shooter/TotalRot", totalRot);
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

        /**
         * Returns the total cumulative rotations of the turret since the subsystem was
         * initialized.
         */
        public double getRotation() {
                return totalRot;
        }

        private void setShooterPower(double power) {
                shootingMotor.set(power);
        }

        private void setTurnPower(double power) {
                if (totalRot >= MaxRot && power > 0) return;
                if (totalRot <= MinRot && power < 0) return;
                turretMotor.set(power);
        }

        public Command runShooterCommand() {
                return this.startEnd(
                                () -> this.setShooterPower(
                                                ShooterSetpoints.kForward),
                                () -> this.setShooterPower(0.0));

        }

        public Command reverseShooterCommand() {
                return this.startEnd(
                                () -> this.setShooterPower(ShooterSetpoints.kReverse),
                                () -> this.setShooterPower(0.0));
        }

        public Command posTurretCommand() {
                return this.startEnd(
                                () -> this.setTurnPower(TurretSetpoints.kPos), () -> this.setTurnPower(0.0));

        }

        public Command negTurretCommand() {

                return this.startEnd(
                                () -> this.setTurnPower(TurretSetpoints.kNeg), () -> this.setTurnPower(0.0));
        }

}