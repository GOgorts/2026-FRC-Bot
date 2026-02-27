package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants.TurningSetpoints;

public class TurningSubsystem extends SubsystemBase {
    private static final int turningMotorCanID = 20;

    private SparkMax turningMotor = new SparkMax(turningMotorCanID, MotorType.kBrushless);

    public TurningSubsystem() {

        turningMotor.configure(
                Configs.ShooterSubsystem.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void setTurningPower(double power) {
        turningMotor.set(power);
    }

    public Command runTurningCommand() {
        return this.startEnd(
                () -> this.setTurningPower(TurningSetpoints.kturnForawrd),
                () -> this.setTurningPower(0.0));
    }

    public Command reverseTurningCommand() {
        return this.startEnd(
                () -> this.setTurningPower(TurningSetpoints.ktunReverse),
                () -> this.setTurningPower(0.0));
    }
}
