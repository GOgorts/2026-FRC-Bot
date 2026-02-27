package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
//import frc.robot.Constants.CoralSubsystemConstants;
//import frc.robot.Constants.CoralSubsystemConstants.ArmSetpoints;
//import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
//import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.Constants.ShooterSubsystemConstants.TurningSetpoints;

public class TurningSubsystem extends SubsystemBase {
        private static final int turningMotorCanID = 20;

        private SparkMax turningMotor = new SparkMax(turningMotorCanID, MotorType.kBrushless);

        public TurningSubsystem() {

                turningMotor.configure(
                                Configs.ShooterSubsystem.turningConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

        }

        private void setTurningPower(double power) {
                turningMotor.set(power);
        }

        public Command runTurningCommand() {
                return new SequentialCommandGroup(
                                new WaitCommand(1)
                                                .andThen(
                                                                this.startEnd(
                                                                                () -> this.setTurningPower(
                                                                                                TurningSetpoints.kturnForawrd),
                                                                                () -> this.setTurningPower(0.0))));

        }

        public Command reverseTurningCommand() {
                return new SequentialCommandGroup(
                                new WaitCommand(1)
                                                .andThen(
                                                                this.startEnd(
                                                                                () -> this.setTurningPower(
                                                                                                TurningSetpoints.ktunReverse),
                                                                                () -> this.setTurningPower(0.0))));

        }
}
