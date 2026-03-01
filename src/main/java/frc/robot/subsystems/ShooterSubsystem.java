package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
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

public class ShooterSubsystem extends SubsystemBase {

        private static final int shootingMotorCanID = 15;
        private static final String kForwardRpmKey = "Shooter/ForwardRPM";
        private static final String kReverseRpmKey = "Shooter/ReverseRPM";
        private static final String kManualModeKey = "Shooter/ManualMode";

        private final SparkMax shootingMotor = new SparkMax(shootingMotorCanID, MotorType.kBrushless);
        private final RelativeEncoder shootingEncoder = shootingMotor.getEncoder();
        private final SparkClosedLoopController shootingController = shootingMotor.getClosedLoopController();
        private final DriveSubsystem m_drive;
        private double targetShooterRpm = 0.0;
        private boolean m_isReady = false;

        private static final InterpolatingDoubleTreeMap kDistanceToRpm = new InterpolatingDoubleTreeMap();

        static {
                for (double[] entry : ShooterSubsystemConstants.ShooterRpmMap.kDistanceRpmTable) {
                        kDistanceToRpm.put(entry[0], entry[1]);
                }
        }

        public ShooterSubsystem(DriveSubsystem drive) {
                m_drive = drive;
                shootingMotor.configure(
                                Configs.ShooterSubsystem.ShooterConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                SmartDashboard.putNumber(kForwardRpmKey, ShooterSubsystemConstants.kDefaultShooterRpm);
                SmartDashboard.putNumber(kReverseRpmKey, ShooterSubsystemConstants.kDefaultReverseShooterRpm);
                SmartDashboard.putBoolean(kManualModeKey, false);
        }

        @Override
        public void periodic() {
                double measuredRpm = shootingEncoder.getVelocity();
                double error = Math.abs(measuredRpm - targetShooterRpm);

                if (targetShooterRpm == 0.0) {
                        m_isReady = false;
                } else if (!m_isReady) {
                        m_isReady = error <= ShooterSubsystemConstants.kReadyToleranceRpm;
                } else {
                        // Stay ready through small target shifts; only drop out if error grows
                        // beyond the tolerance + hysteresis band
                        m_isReady = error <= ShooterSubsystemConstants.kReadyToleranceRpm
                                        + ShooterSubsystemConstants.kHysteresisRpm;
                }

                SmartDashboard.putNumber("Shooter/DistanceToHub", getDistanceToHub());
                SmartDashboard.putNumber("Shooter/AutomaticRPM", getRpmForDistance(getDistanceToHub()));
                SmartDashboard.putNumber("Shooter/TargetRPM", targetShooterRpm);
                SmartDashboard.putNumber("Shooter/MeasuredRPM", measuredRpm);
                SmartDashboard.putBoolean("Shooter/IsReady", m_isReady);
        }

        /**
         * Returns true when the shooter is spun up to within tolerance of its target
         * RPM.
         */
        public boolean isShooterReady() {
                return m_isReady;
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
         * Returns interpolated shooter target RPM for a given distance from the hub
         * (meters).
         * Values outside the table range are clamped to the nearest endpoint.
         */
        public double getRpmForDistance(double distanceMeters) {
                return kDistanceToRpm.get(distanceMeters);
        }

        /** Returns true when manual mode is enabled via the dashboard toggle. */
        public boolean isManualMode() {
                return SmartDashboard.getBoolean(kManualModeKey, false);
        }

        /**
         * Returns a command that spins up the shooter using either the
         * distance-interpolated
         * RPM (automatic) or the manual ForwardRPM value from the dashboard, depending
         * on
         * the Shooter/ManualMode toggle. Updating every cycle lets isShooterReady()
         * track
         * the actual target rather than a stale snapshot.
         */
        public Command runShooterAutomaticCommand() {
                return this.run(() -> {
                        double rpm = isManualMode()
                                        ? SmartDashboard.getNumber(kForwardRpmKey,
                                                        ShooterSubsystemConstants.kDefaultShooterRpm)
                                        : -getRpmForDistance(getDistanceToHub());
                        this.setShooterRpm(rpm);
                }).finallyDo(() -> this.setShooterRpm(0.0));
        }

        private void setShooterRpm(double rpm) {
                targetShooterRpm = rpm;
                shootingController.setSetpoint(rpm, ControlType.kVelocity);
        }

        public Command runShooterCommand() {
                return this.startEnd(
                                () -> this.setShooterRpm(SmartDashboard.getNumber(kForwardRpmKey,
                                                ShooterSubsystemConstants.kDefaultShooterRpm)),
                                () -> this.setShooterRpm(0.0));
        }

        public Command reverseShooterCommand() {
                return this.startEnd(
                                () -> this.setShooterRpm(SmartDashboard.getNumber(kReverseRpmKey,
                                                ShooterSubsystemConstants.kDefaultReverseShooterRpm)),
                                () -> this.setShooterRpm(0.0));
        }
}
