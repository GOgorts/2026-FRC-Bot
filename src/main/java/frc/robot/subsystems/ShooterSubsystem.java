package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {

        private static final int shootingMotorCanID = 15;
        private static final String kForwardKey = "Shooter/Forward";
        private static final String kReverseKey = "Shooter/Reverse";

        private SparkMax shootingMotor = new SparkMax(shootingMotorCanID, MotorType.kBrushless);

        public ShooterSubsystem() {
                shootingMotor.configure(
                                Configs.ShooterSubsystem.ShooterConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                SmartDashboard.putNumber(kForwardKey, 0.67);
                SmartDashboard.putNumber(kReverseKey, -0.67);
        }

        private void setShooterPower(double power) {
                shootingMotor.set(power);
        }

        public Command runShooterCommand() {
                return this.startEnd(
                                () -> this.setShooterPower(SmartDashboard.getNumber(kForwardKey, 0.67)),
                                () -> this.setShooterPower(0.0));
        }

        public Command reverseShooterCommand() {
                return this.startEnd(
                                () -> this.setShooterPower(SmartDashboard.getNumber(kReverseKey, -0.67)),
                                () -> this.setShooterPower(0.0));
        }
}
