package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
//import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
//import frc.robot.Constants.AlgaeSubsystemConstants.flip1Setpoints;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.flipSetpoints;

public class IntakeSubsystem extends SubsystemBase{

    public enum Setpoint {
    kUp, kDown;
  }
    private boolean wasResetByLimit = false;
    public static final int kIntakeMotorCanId = 9;
    private static final int kIntakeFlipCanID1 = 10;
    private static final int kIntakeFlipCanID2 = 11;
    public IntakeSubsystem(){
        //flipMotor1 & 2 were IntakeConfig (changed to FlipConfig)
        intakeMotor.configure(
            Configs.IntakeSubsystem.IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flipMotor1.configure(
            Configs.IntakeSubsystem.FlipLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flipMotor2.configure(
            Configs.IntakeSubsystem.FlipFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flipEncoder1.setPosition(0);
        flipEncoder2.setPosition(0);

        initializeFlip();
        
    }
    private SparkFlex intakeMotor = 
        new SparkFlex(kIntakeMotorCanId, MotorType.kBrushless);
    private SparkMax flipMotor1 = 
        new SparkMax(kIntakeFlipCanID1,MotorType.kBrushless);
    private SparkMax flipMotor2 = 
        new SparkMax(kIntakeFlipCanID2,MotorType.kBrushless);
    private RelativeEncoder flipEncoder1 = flipMotor1.getEncoder();
    private RelativeEncoder flipEncoder2 = flipMotor2.getEncoder();

    private SparkClosedLoopController flip1Controller = flipMotor1.getClosedLoopController();
    private SparkClosedLoopController flip2Controller = flipMotor2.getClosedLoopController();

    private double flipCurrent = flipSetpoints.kUp;
    
    private void zeroIntakeOnLimitSwitch() {
    if (!wasResetByLimit && flipMotor1.getReverseLimitSwitch().isPressed()&&flipMotor2.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      flipEncoder1.setPosition(0);
      flipEncoder2.setPosition(0);
      wasResetByLimit = true;

    }
    else if (!flipMotor1.getReverseLimitSwitch().isPressed()
    &&flipMotor2.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

    public void initializeFlip() {
    // Check if limit switches are pressed at startup
    if (flipMotor1.getReverseLimitSwitch().isPressed() &&
        flipMotor2.getReverseLimitSwitch().isPressed()) {
        
        flipEncoder1.setPosition(0);
        flipEncoder2.setPosition(0);
        wasResetByLimit = true;
        System.out.println("Flip encoder zeroed on startup");
    } else {
        System.out.println("Flip not at bottom, do not zero yet");
    }
}

    

    private void setIntakePower (double power) {
         intakeMotor.set(power);
    }
    private void setFlipSetpoint (){
         //multiplied by -1 to ensure motor is inverted from Motor1
        flip1Controller.setSetpoint(flipCurrent, ControlType.kMAXMotionPositionControl);
        flip2Controller.setSetpoint(flipCurrent*-1, ControlType.kMAXMotionPositionControl);
        
    }

    public Command flipUpCommand() {
    return this.runOnce(() -> {
        flipCurrent = flipSetpoints.kIn;
        setFlipSetpoint();
        System.out.println("FlipUp ");
    });
}

public Command flipDownCommand() {
    return this.runOnce(() -> {
        flipCurrent = flipSetpoints.kUp;
        setFlipSetpoint();
                System.out.println("FlipDown ");

    });
}

    
    public Command runIntakeCommand() {
        
            return this.startEnd(
                () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.0));
                 }
    
    public Command reverseIntakeCommand() {
            return this.startEnd(
                () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
                 }


    public void periodic()
    {
       // zeroIntakeOnLimitSwitch();
    }

    }    
    

   
  

