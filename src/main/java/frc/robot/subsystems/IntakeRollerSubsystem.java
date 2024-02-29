package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.IntakeRollerConstants;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

/**
 * The IntakeSubsystem class represents a subsystem for controlling the intake
 * mechanism of a robot.
 * It includes functionality for intake and outtake operations, positioning the
 * intake, retrieving the current position,
 * checking if the intake is at the required position, and detecting the
 * presence of an object on the intake.
 */
public class IntakeRollerSubsystem extends SubsystemBase {
    /** Motor controller for controlling the intake. */
    CANSparkMax intakeMotor;

    /** PID controller for maintaining the turning motor position. */
    SparkPIDController pidController;

    /** Flag indicating whether the intake is outside or not. */
    boolean outside;

    /**
     * Creates a new IntakeSubsystem with initialized motor controllers and PID
     * controller.
     */
    public IntakeRollerSubsystem() {

        // Initialization of motor controllers and PID controller
        intakeMotor = new CANSparkMax(IntakeRollerConstants.kIntakeID, MotorType.kBrushless);
        intakeMotor.setInverted(true);
    

    }

    
    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * Checks if an object is detected on the intake based on the current output
     * current.
     *
     * @return True if an object is detected on the intake; false otherwise.
     */
    public boolean objectOnHand() {
        if (intakeMotor.getOutputCurrent() >= IntakeRollerConstants.kIntakeCurrentLimit) {
            return true;
        }
        return false;
    }
    

    /**
     * COMMANDS
     */

    /**
     * Creates an InstantCommand for outtake operation.
     *
     * @return An InstantCommand object for outtake operation.
     */

    public Command stopCommand() {
        return setSpeedCommand(IntakeRollerConstants.kStop);
    }
    public Command intakeSenseCommand() {
        return setSpeedCommand(IntakeRollerConstants.kIntake).until(() -> objectOnHand());
    }
    public Command dumbIntakeCommand(){
        return setSpeedCommand(IntakeRollerConstants.kIntake).withTimeout(0.1);

    }
    public Command intakeCommand(){
        return dumbIntakeCommand().andThen(intakeSenseCommand());
    }
    public Command outtakeCommand() {
        return setSpeedCommand(IntakeRollerConstants.kOuttake);
    }
    public Command holdCommand() {
        return setSpeedCommand(IntakeRollerConstants.kHold);
    }
    public Command setSpeedCommand(double speed){
        return run(() -> setSpeed(speed));
    }
}