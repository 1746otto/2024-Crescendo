package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

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

    }

    /**
     * Sets the intake motor speed for intake operation.
     *
     * @param speed The speed at which the intake motor should operate.
     */
    public void intake(double speed) {
            intakeMotor.set(speed);
    }

    private void stopIntake() {
        intakeMotor.set(0);
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

    public Command stop() {
        return run(() -> stopIntake());
    }
    public Command basicIntake() {
        return run(() -> intake(IntakeRollerConstants.kIntakeSpeed));
    }
    public Command outtake() {
        return run(() -> intake(IntakeRollerConstants.kOuttakeSpeed));
    }
    public Command hold() {
        return run(() -> intake(IntakeRollerConstants.kIntakeHoldSpeed));
    }
    public Command intakeWithCurrentSensing() {
        return run(() -> intake(IntakeRollerConstants.kIntakeSpeed)).until(() -> objectOnHand());
    }
    public Command outtakeWithCurrentSensing() {
        return run(() -> intake(-IntakeRollerConstants.kIntakeSpeed)).until(() -> !objectOnHand());
    }
}