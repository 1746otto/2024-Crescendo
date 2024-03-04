package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.PrimerConstants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;


/**
 * Class for PrimerSubsystem to move game pieces from indexer to a holding space near the shooter.
 */
public class PrimerSubsystem extends SubsystemBase{
  /** CANSparkMax motor controller for the priming roller. */
  private TalonSRX primerNeo;
  private AnalogInput speakerBeamBreak;
  private AnalogInput ampBeamBreak;

  /** CANSparkMan pid controller */
  //private SparkPIDController pidController;

  /**
   * Creates a new PrimerSubsystem with initialized motor controller.
   */
  public PrimerSubsystem() {
    primerNeo = new TalonSRX(PrimerConstants.kPrimerRollerMotorID);
    primerNeo.setInverted(true);
    primerNeo.setNeutralMode(NeutralMode.Brake);
    speakerBeamBreak = new AnalogInput(PrimerConstants.kPrimerSpeakerBeamBreakID);
    // pidController.setP(PrimerConstants.kP);
    // pidController.setI(PrimerConstants.kI);
    // pidController.setFF(PrimerConstants.kFF);
    ampBeamBreak = new AnalogInput(PrimerConstants.kPrimerAmpBeamBreakID);

  }

  /**
   * Sets the priming roller speed for priming the shooting mechanism.
   */
  public void primeNote(double rpm) {
    //pidController.setReference(rpm, ControlType.kVelocity);
  }
  
  public void setSpeed(double speed) {
    primerNeo.set(ControlMode.PercentOutput,speed);
  }
  /**
   * Run the motor backwards at a slow speed of kPrimerReverseSpeed.
   */
  
  public void returnNote() {
    primerNeo.set(ControlMode.PercentOutput,PrimerConstants.kOuttake);
  }
  public void note() {
    primerNeo.set(ControlMode.PercentOutput,PrimerConstants.kIntake);
  }
  /**
   * Stops the primer motor
   */
  public void stopPrimer() {
    primerNeo.set(ControlMode.PercentOutput,PrimerConstants.kStop);
  }

  /**
   * Checks if the game piece is pinched into the primer at a certain point.
   * @return true or false if game piece is held well in primer.
   */
  public boolean isObjectPinchedInPrimer() {
    return (Math.floor(speakerBeamBreak.getVoltage()) == 0);
  }
  public boolean isObjectReadyToAmp() {
    return (Math.floor(ampBeamBreak.getVoltage()) == 0);
  }

  /** Returns the velocity of the motor */
  // public double getVelocity() {
  //   return pidController.getSmartMotionMaxVelocity(PrimerConstants.kPrimerSlotID);
  // }



  public Command ampReadyCommand() {
    return setSpeedCommand(PrimerConstants.kOuttake).until(() -> isObjectReadyToAmp()).finallyDo(() -> setSpeedCommand(0));
  }
  public Command ampCommand() {
    return setSpeedCommand(PrimerConstants.kAmp);
  }
  public Command intakeCommand() {
    return setSpeedCommand(PrimerConstants.kIntake).until(() -> isObjectPinchedInPrimer()).finallyDo(() -> setSpeedCommand(0));
  }
  public Command outtakeCommand() {
    return setSpeedCommand(PrimerConstants.kOuttake).until(() -> !isObjectPinchedInPrimer()).finallyDo(() -> setSpeedCommand(0));
  }
  public Command shootCommand() {
    return setSpeedCommand(PrimerConstants.kShoot).until(() -> !isObjectPinchedInPrimer()).finallyDo(() -> setSpeedCommand(0));
  }
  public Command stopCommand() {
    return setSpeedCommand(PrimerConstants.kStop);
  }

  public Command setSpeedCommand(double speed) {
    return run(() -> setSpeed(speed));
  }
  @Override
  public void periodic() {
    //System.out.println(isObjectPinchedInPrimer()); //To change
  }
  
}
