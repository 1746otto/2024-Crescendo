package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PrimerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class PrimerSubsystem extends SubsystemBase{
    /** CANSparkMax motor controller for the priming roller. */
    private CANSparkMax primerNeo;

    private double primingSpeed = frc.robot.Constants.PrimerConstants.kPrimerRollerSpeed;

    public PrimerSubsystem(){
        primerNeo = new CANSparkMax(PrimerConstants.kPrimerRollerMotorID, MotorType.kBrushless);
    }

  /**
   * Sets the priming roller speed for priming the shooting mechanism.
   */
  public void primeNote() {
    primerNeo.set(primingSpeed);
  }
  public void stop(){
    primerNeo.set(0.0);
  }

  public Command PrimeCommand(){
    return new InstantCommand(() -> primeNote());
  }
  public Command StopCommand(){
    return new InstantCommand(() -> stop());
  }
}
