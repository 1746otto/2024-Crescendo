package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.PrimerConstants;
import edu.wpi.first.wpilibj.AnalogInput;
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
  private CANSparkMax primerNeo;

  /** CANSparkMan pid controller */
  private SparkPIDController pidController;

  /** Analog input for detecting beam breaks. */
  private AnalogInput beamBreak;

  /**
   * Creates a new PrimerSubsystem with initialized motor controller.
   */
  public PrimerSubsystem() {
    primerNeo = new CANSparkMax(PrimerConstants.kPrimerRollerMotorID, MotorType.kBrushless);
    // Initialization of analog input for beam break detection
    beamBreak = new AnalogInput(ShooterConstants.kShooterAnalogInputChannel);
    pidController = primerNeo.getPIDController();
    pidController.setP(PrimerConstants.kP);
    pidController.setI(PrimerConstants.kI);
    pidController.setFF(PrimerConstants.kFF);

  }

  /**
   * Sets the priming roller speed for priming the shooting mechanism.
   */
  public void primeNote(double rpm) {
    pidController.setReference(rpm, ControlType.kVelocity);
  }
  /**
   * Run the motor backwards at a slow speed of kPrimerReverseSpeed.
   */
  public void returnNote() {
    primerNeo.set(PrimerConstants.kPrimerReverseSpeed);
  }
  /**
   * Stops the primer motor
   */
  public void stopPrimer() {
    primerNeo.set(PrimerConstants.kPrimerStopSpeed);
  }

  /**
   * Checks if the game piece is pinched into the primer at a certain point.
   * @return true or false if game piece is held well in primer.
   */
  public boolean isObjectPinchedInPrimer() {
    return (primerNeo.getOutputCurrent() >= PrimerConstants.kPrimerCurrentLimit);
  }

  /**
   * Returns a BooleanSupplier representing if the beambreak has been broken or not.
   */
  public BooleanSupplier isBeamBreakBroken() {
    return () -> ((Math.floor(beamBreak.getVoltage()) > 0));
  }

  /** Returns the velocity of the motor */
  public double getVelocity() {
    return pidController.getSmartMotionMaxVelocity(PrimerConstants.kPrimerSlotID);
  }




  /**
   * Command to run the primer forwards at a certain speed to move piece into holding space
   * @return
   */
  public Command PrimeCommand(double rpm) {
    return run(() -> primeNote(rpm));
  }
  /**
   * Command to run the primer backwards if the game piece ends up too far in holding space.
   * @return
   */
  public Command ReverseCommand() {
    return run(this::returnNote);
  }
  /**
   * Command to stop the primer from running.
   * @return
   */
  public Command StopCommand() {
    return run(this::stopPrimer);
  }
}
