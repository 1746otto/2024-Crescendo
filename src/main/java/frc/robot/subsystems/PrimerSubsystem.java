package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.PrimerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



public class PrimerSubsystem extends SubsystemBase{
    /** CANSparkMax motor controller for the priming roller. */
    private CANSparkMax primerNeo;

  /**
   * Creates a new PrimerSubsystem with initialized motor controller.
   */
    public PrimerSubsystem(){
        primerNeo = new CANSparkMax(PrimerConstants.kPrimerRollerMotorID, MotorType.kBrushless);
    }

  /**
   * Sets the priming roller speed for priming the shooting mechanism.
   */
  public void primeNote() {
    primerNeo.set(PrimerConstants.kPrimerRollerSpeed);
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
  public void stopPrimer(){
    primerNeo.set(PrimerConstants.kPrimerStopSpeed);
  }

  /**
   * Checks if the game piece is pinched into the primer at a certain point.
   * @return true or false if game piece is held well in primer.
   */
  public boolean isObjectPinchedInPrimer(){
    return (primerNeo.getOutputCurrent() >= PrimerConstants.kPrimerCurrentLimit);
  }


  /**
   * Command to run the primer forwards to move piece into holding space
   * @return
   */
  public Command PrimeCommand(){
    return run(() -> primeNote());
  }
  /**
   * Command to run the primer backwards if the game piece ends up too far in holding space.
   * @return
   */
  public Command ReverseCommand(){
    return run(() -> returnNote());
  }
  /**
   * Command to stop the primer from running.
   * @return
   */
  public Command StopCommand(){
    return run(() -> stopPrimer());
  }
}
