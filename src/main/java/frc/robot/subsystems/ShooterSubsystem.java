package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AllianceHelper;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.ShooterSubsystemConstants.TurretTracking;
import frc.robot.subsystems.DriveSubsystem;

public class ShooterSubsystem extends SubsystemBase {

        private static final int shootingMotorCanID = 15;
        private static final String kForwardKey = "Shooter/Forward";
        private static final String kReverseKey = "Shooter/Reverse";

        private SparkMax shootingMotor = new SparkMax(shootingMotorCanID, MotorType.kBrushless);
        private final DriveSubsystem m_drive;

        private static final InterpolatingDoubleTreeMap kDistanceToPower = new InterpolatingDoubleTreeMap();

        static {
                for (double[] entry : ShooterSubsystemConstants.ShooterPowerMap.kDistancePowerTable) {
                        kDistanceToPower.put(entry[0], entry[1]);
                }
        }

        public ShooterSubsystem(DriveSubsystem drive) {
                m_drive = drive;
                shootingMotor.configure(
                                Configs.ShooterSubsystem.ShooterConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                SmartDashboard.putNumber(kForwardKey, ShooterSubsystemConstants.kDefaultShooterPower);
                SmartDashboard.putNumber(kReverseKey, -0.67);
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("Shooter/DistanceToHub", getDistanceToHub());
        }

        /**
         * Returns the robot's current straight-line distance to the alliance hub
         * (meters).
         */
        public double getDistanceToHub() {
                Translation2d hubPos = AllianceHelper.isRedAlliance()
                                ? TurretTracking.kRedHubCenter
                                : TurretTracking.kBlueHubCenter;
                Translation2d robotPos = m_drive.getPose().getTranslation();
                return robotPos.getDistance(hubPos);
        }

        /**
         * Returns interpolated shooter motor power for a given distance from the hub
         * (meters).
         * Values outside the table range are clamped to the nearest endpoint.
         */
        public double getPowerForDistance(double distanceMeters) {
                return kDistanceToPower.get(distanceMeters);
        }

        /**
         * Returns a command that runs the shooter at the interpolated power for the
         * current hub distance.
         */
        public Command runShooterAutomaticCommand() {
                return this.startEnd(
                                () -> this.setShooterPower(getPowerForDistance(getDistanceToHub())),
                                () -> this.setShooterPower(0.0));
        }

        private void setShooterPower(double power) {
                shootingMotor.set(power);
        }

        public Command runShooterCommand() {
                return this.startEnd(
                                () -> this.setShooterPower(SmartDashboard.getNumber(kForwardKey,
                                                ShooterSubsystemConstants.kDefaultShooterPower)),
                                () -> this.setShooterPower(0.0));
        }

        public Command reverseShooterCommand() {
                return this.startEnd(
                                () -> this.setShooterPower(SmartDashboard.getNumber(kReverseKey, -0.67)),
                                () -> this.setShooterPower(0.0));
        }
}
