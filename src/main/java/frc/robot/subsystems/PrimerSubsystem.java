package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PrimerConstants;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * Class for PrimerSubsystem to move game pieces from indexer to a holding space near the shooter.
 */
public class PrimerSubsystem extends SubsystemBase{
  /** CANSparkMax motor controller for the priming roller. */
  private TalonSRX primerNeo;

  private boolean ampMode;

  /**
   * Creates a new PrimerSubsystem with initialized motor controller.
   */
  public PrimerSubsystem() {
    ampMode = false;

    primerNeo = new TalonSRX(PrimerConstants.kPrimerRollerMotorID);
    // pidController.setP(PrimerConstants.kP);
    // pidController.setI(PrimerConstants.kI);
    // pidController.setFF(PrimerConstants.kFF);

  }

  /**
   * Sets the priming roller speed for priming the shooting mechanism.
   */
  public void primeNote(double rpm) {
    //pidController.setReference(rpm, ControlType.kVelocity);
  }
  /**
   * Run the motor backwards at a slow speed of kPrimerReverseSpeed.
   */
  public void returnNote() {
    primerNeo.set(ControlMode.PercentOutput,PrimerConstants.kPrimerReverseSpeed);
  }
  public void note() {
    primerNeo.set(ControlMode.PercentOutput,PrimerConstants.kPrimerRollerSpeed);
  }
  /**
   * Stops the primer motor
   */
  public void stopPrimer() {
    primerNeo.set(ControlMode.PercentOutput,PrimerConstants.kPrimerStopSpeed);
  }

  /**
   * Checks if the game piece is pinched into the primer at a certain point.
   * @return true or false if game piece is held well in primer.
   */
  public boolean isObjectPinchedInPrimer() {
    return (primerNeo.getOutputCurrent() >= PrimerConstants.kPrimerCurrentLimit);
  }

  public boolean isNoteInShotPosition() {
    // Placeholder. Needs beambreak
    return false;
  }

  private void toggleAmp() {
    this.ampMode = !ampMode;
  }



  // ======================================
  // ==============Commands================
  // ======================================
  /**
   * Command to run the primer forwards at a certain speed to move piece into holding space
   * @return
   */
  public Command setToPrime(double rpm) {
    return run(() -> primeNote(rpm));
  }
  public Command setToForward() {
    return run(this::note);
  }
  /**
   * Command to run the primer backwards if the game piece ends up too far in holding space.
   * @return
   */
  public Command setToReverse() {
    return run(this::returnNote);
  }
  /**
   * Command to stop the primer from running.
   * @return
   */
  public Command stop() {
    return run(this::stopPrimer);
  }
  public Command toggleAmpMode() {
    return runOnce(() -> toggleAmp());
  }
  public Command goToSetPose() {
    // Need to determine logic here with beambreaks
    return runOnce(() -> {
      if (ampMode) {

      }
      else {

      }
    });
  }
}
