package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
//import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
    public static final FeedForwardConfig drivingFF = new FeedForwardConfig();
    public static final ClosedLoopConfig DRIV_CLOSED_LOOP_CONFIG = new ClosedLoopConfig();
    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          // .velocityFF(drivingVelocityFeedForward)
          // drivingFF.kVelocity(drivingVelocityFeedForward,0)
          .outputRange(-1.0, 1.0).feedForward.kV(drivingVelocityFeedForward);

      turningConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(20);
      turningConfig.absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class IntakeSubsystem {
    public static final SparkFlexConfig IntakeConfig = new SparkFlexConfig();
    static {
      IntakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    }
    public static final SparkMaxConfig FlipLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig FlipFollowerConfig = new SparkMaxConfig();

    static {
      FlipLeaderConfig
          .inverted(false)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(40);

      FlipLeaderConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.04, 0, 0)
          .outputRange(-1, 1);

      FlipLeaderConfig.closedLoop.maxMotion
          .cruiseVelocity(3000)
          .maxAcceleration(4000)
          .allowedProfileError(0.5);

      FlipFollowerConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(40);
    }
  }

  public static final class ShooterSubsystem {
    public static final SparkMaxConfig ShooterConfig = new SparkMaxConfig();
    static {
      ShooterConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    }
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
    static {
      turningConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    }
    public static final SparkMaxConfig TurretConfig = new SparkMaxConfig();
    static {
      TurretConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
      TurretConfig.absoluteEncoder.positionConversionFactor(1.0);
    }

  }

  public static final class CoralSubsystem {
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the arm motor
      armConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.01)
          .outputRange(-1, 1).maxMotion
          // Set MAXMotion parameters for position control
          .cruiseVelocity(2000)
          .maxAcceleration(10000)
          .allowedProfileError(0.25);

      // Configure basic settings of the elevator motor
      elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit
       * switch, this
       * will prevent any actuation of the elevator in the reverse direction if the
       * limit switch is
       * pressed.
       */
      elevatorConfig.limitSwitch
          .reverseLimitSwitchTriggerBehavior(null)
          .reverseLimitSwitchType(Type.kNormallyOpen);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1).maxMotion
          // Set MAXMotion parameters for position control
          .cruiseVelocity(4200)
          .maxAcceleration(6000)
          .allowedProfileError(0.5);

      // Configure basic settings of the intake motor
      intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    }
  }

  public static final class AlgaeSubsystem {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();

    static {
      // Configure basic setting of the arm motor
      armConfig.smartCurrentLimit(40);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed
          // loop slot, as it will default to slot 0.
          .p(0.1)
          .outputRange(-0.5, 0.5);

      // Configure basic settings of the intake motor
      intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    }
  }

  public static final class HangSubsystem {
    public static final SparkMaxConfig hangConfig = new SparkMaxConfig();

    static {

      // Configure basic settings of the hang motor
      hangConfig
          .inverted(false)
          .idleMode(IdleMode.kBrake);
    }
  }
}
