package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
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
public class IntakeSubsystem extends SubsystemBase {

    /** Motor controller for turning the intake mechanism. */
    CANSparkMax turningMotor;

    /** Motor controller for controlling the intake. */
    CANSparkMax intakeMotor;

    /** PID controller for maintaining the turning motor position. */
    SparkPIDController pidController;

    /** Flag indicating whether the intake is outside or not. */
    boolean outside;

    /** The required position for the turning motor. */
    double reqPosition;

    /**
     * Creates a new IntakeSubsystem with initialized motor controllers and PID
     * controller.
     */
    public IntakeSubsystem() {

        // Initialization of motor controllers and PID controller
        turningMotor = new CANSparkMax(IntakeConstants.kIntakeTurnID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeID, MotorType.kBrushless);
        pidController = turningMotor.getPIDController();
        pidController.setP(IntakeConstants.kP);
        pidController.setOutputRange(-.2, .2);

        // Setting the initial required position to the origin
        reqPosition = IntakeConstants.kOriginPosition;

    }

    /**
     * Sets the intake motor speed for intake operation.
     *
     * @param speed The speed at which the intake motor should operate.
     */
    public void intake(double speed) {
            intakeMotor.set(speed);
    }

    /**
     * Sets the intake motor speed for outtake operation.
     */
    public void outtake() {
        intakeMotor.set(-IntakeConstants.kIntakeSpeed);
    }

    /**
     * Sets the target position for the turning motor using PID control.
     *
     * @param req The target position for the turning motor.
     */
    public void intakeToReq(double req) {
        pidController.setReference(req, ControlType.kPosition);
    }

    /**
     * Sets the required position for the turning motor.
     *
     * @param req The required position for the turning motor.
     */
    public void setRequest(double req) {
        reqPosition = req;
    }

    /**
     * Gets the current position of the turning motor.
     *
     * @return The current position of the turning motor.
     */
    public double getPosition() {
        return turningMotor.getEncoder().getPosition();
    }

    /**
     * Checks if the turning motor is at the required position within a specified
     * tolerance.
     *
     * @param reqPos The required position to check against.
     * @return True if the turning motor is at the required position; false
     *         otherwise.
     */
    public boolean isAtReqPosition(double reqPos) {

        if ((getPosition() >= (reqPos - IntakeConstants.kTolerance))
                && (getPosition() <= (reqPos + IntakeConstants.kTolerance))) {
            return true;
        }
        return false;
    }

    /**
     * Checks if an object is detected on the intake based on the current output
     * current.
     *
     * @return True if an object is detected on the intake; false otherwise.
     */
    public boolean objectOnHand() {
        if (intakeMotor.getOutputCurrent() >= IntakeConstants.kIntakeCurrentLimit) {
            return true;
        }
        return false;
    }

    /**
     * COMMANDS
     */

    /**
     * Creates a command for intake operation until an object is detected and then
     * transitions to stow position.
     *
     * @return A Command object for intake operation.
     */
    public Command IntakeCommand() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(run(() -> intake(IntakeConstants.kIntakeSpeed))
            .until(() -> objectOnHand() == true),
            runOnce(() -> setRequest(IntakeConstants.kOutPosition))),
            runOnce(() -> setRequest(IntakeConstants.kOriginPosition)),
            run(() -> intake(IntakeConstants.kItakeStowSpeed))
            .until(() -> objectOnHand() == false)
             );
    }

    /**
     * Creates an InstantCommand for outtake operation.
     *
     * @return An InstantCommand object for outtake operation.
     */
    public InstantCommand OuttakeCommand() {
        return new InstantCommand(() -> outtake());
    }

    /**
     * Periodic method for updating the turning motor's position based on the
     * required position.
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run


        intakeToReq(reqPosition);
    }
}