package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants.ShooterSetpoints;

public class ShooterSubsystem extends SubsystemBase {

        private static final int shootingMotorCanID = 15;

        private SparkMax shootingMotor = new SparkMax(shootingMotorCanID, MotorType.kBrushless);

        public ShooterSubsystem() {
                shootingMotor.configure(
                                Configs.ShooterSubsystem.ShooterConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
        }

        private void setShooterPower(double power) {
                shootingMotor.set(power);
        }

        public Command runShooterCommand() {
                return this.startEnd(
                                () -> this.setShooterPower(ShooterSetpoints.kForward),
                                () -> this.setShooterPower(0.0));
        }

        public Command reverseShooterCommand() {
                return this.startEnd(
                                () -> this.setShooterPower(ShooterSetpoints.kReverse),
                                () -> this.setShooterPower(0.0));
        }
}
