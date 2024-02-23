package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import frc.robot.Constants.IntakeConstants;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private CANSparkMax positionMotor;

    /** Motor controller for controlling the intake. */
    private CANSparkMax intakeMotor;

    /** PID controller for maintaining the turning motor position. */
    private SparkPIDController pidController;

    /** Flag indicating whether the intake is outside or not. */
    private boolean isOutside;

    /** The requested position for the turning motor. */
    private double reqPosition;

    /**
     * Creates a new IntakeSubsystem with initialized motor controllers and PID
     * controller.
     */
    public IntakeSubsystem() {
        // Initialization of motor controllers and PID controller
        positionMotor = new CANSparkMax(IntakeConstants.kIntakeWristID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeID, MotorType.kBrushless);
        intakeMotor.setInverted(IntakeConstants.kIntakeState);
        pidController = positionMotor.getPIDController();
        pidController.setP(IntakeConstants.kP);
        pidController.setFF(IntakeConstants.kFF);
        pidController.setOutputRange(IntakeConstants.kTestingOutputMin, IntakeConstants.kTestingOutputMax);
        positionMotor.getEncoder().setPosition(IntakeConstants.kOriginPosition);
        // pidController.setFeedbackDevice();

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
     * Position motor runs to the current requested position, using PID control.
     *
     * @param req The requested position for the position motor.
     */
    public void runToRequest(double req) {
        pidController.setReference(req, ControlType.kPosition);
    }

    /**
     * Requested a position for the position motor to go.
     *
     * @param req The requested position for the position motor.
     */
    public void setRequest(double req) {
        reqPosition = req;
    }

    /**
     * Gets the current position of the position motor.
     *
     * @return The current position of the position motor.
     */
    public double getPosition() {
        return positionMotor.getEncoder().getPosition();
    }

    /**
     * Checks if the position motor is at the requested position within a specified
     * tolerance.
     *
     * @param reqPos The requested position to check against.
     * @return True if the position motor is at the requested position; false
     *         otherwise.
     */
    public boolean isAtReqPosition(double reqPos) {

        return ((getPosition() >= (reqPos - IntakeConstants.kTolerance))
                && (getPosition() <= (reqPos + IntakeConstants.kTolerance)));
    }

    /**
     * Checks if an object is detected on the intake based on the current output
     * current.
     *
     * @return True if an object is detected on the intake; false otherwise.
     */
    public boolean isObjectOnHand() {
        return (intakeMotor.getOutputCurrent() >= IntakeConstants.kIntakeCurrentLimit);
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
    public Command intakeWithCurrentSensingCommand() {
        return new SequentialCommandGroup(
            setRequestPositionToGroundCommand(),
            run(() -> intake(IntakeConstants.kIntakeSpeed)).until(() -> isObjectOnHand()),
            runOnce(() -> intake(IntakeConstants.kIntakeStopSpeed)),
            runOnce(() -> setRequest(IntakeConstants.kOriginPosition))
        );
    }

    /**
     * Creates a command for intake operation until command is told to stop and then moves to stow position.
     * @return
     */
    public Command basicIntakeCommand() {
        return new StartEndCommand(() -> {
            setRequest(IntakeConstants.kOutakePosition);
            intake(IntakeConstants.kIntakeSpeed);
            }
            , () -> {
            setRequest(IntakeConstants.kOriginPosition);
            intake(IntakeConstants.kIntakeStopSpeed);
            }, 
            this);
    }
    public Command basicTest() {
        return run(() -> setRequest(IntakeConstants.kOutakePosition)).finallyDo(() -> setRequest(IntakeConstants.kOriginPosition));
    }

    /**
     * Command to request position motor to head to the origin position
     * @return
     */
    public Command setRequestPositionToGroundCommand() {
        return runOnce(() -> setRequest(IntakeConstants.kOutakePosition));
    }
    

    /**
     * Command to run the intake motor backwards.
     *
     * @return
     */
    public Command outtakeCommand() {
        return runOnce(() -> intake(IntakeConstants.kIntakeRevSpeed));
    }

    /**
     * COmmand to stop the intake motor.
     * @return
     */
    public Command stopIntakingCommand() {
        return runOnce(() -> intake(IntakeConstants.kIntakeStopSpeed));
    }

    /**
     * Periodic method for updating the turning motor's position based on the
     * required position.
     */
    @Override
    public void periodic() {    
        // This method will be called once per scheduler run
        SmartDashboard.putNumber(IntakeConstants.kIntakePosLabel, getPosition());
        SmartDashboard.putNumber("Intake current", intakeMotor.getOutputCurrent());
        runToRequest(reqPosition);
    }
}