package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.IntakeWristConstants;

public class IntakeWristSubsystem extends SubsystemBase{
    /** Motor controller for turning the intake mechanism. */
    CANSparkMax turningMotor;

    /** PID controller for maintaining the turning motor position. */
    SparkPIDController pidController;

    /** The required position for the turning motor. */
    double reqPosition;
    double current;

    /**
     * Creates a new IntakeSubsystem with initialized motor controllers and PID
     * controller.
     */
    public IntakeWristSubsystem() {

        // Initialization of motor controllers and PID controller
        turningMotor = new CANSparkMax(IntakeWristConstants.kIntakeTurnID, MotorType.kBrushless);
        pidController = turningMotor.getPIDController();
        pidController.setP(IntakeWristConstants.kP);
        pidController.setOutputRange(-.2, .2);

        // Setting the initial required position to the origin
        turningMotor.getEncoder().setPosition(IntakeWristConstants.kStow);
        turningMotor.setIdleMode(IdleMode.kBrake);
        reqPosition = IntakeWristConstants.kStow;
        

    }
    public boolean isCurrentMax() {
        if (turningMotor.getOutputCurrent() >= 40) {
            return true;
        }
        return false;
    }

    public void testIntake() {
        turningMotor.set(0.1);
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

        if ((getPosition() >= (reqPos - IntakeWristConstants.kTolerance))
                && (getPosition() <= (reqPos + IntakeWristConstants.kTolerance))) {
            return true;
        }
        return false;
    }

    /**
     * COMMANDS
     */
    public Command IntakePositionCheck(double requestedPos) {
        return run(() -> isAtReqPosition(requestedPos)).until(() -> isAtReqPosition(requestedPos) == true);
    }

    /**
     * Periodic method for updating the turning motor's position based on the
     * required position.
     */
    public Command indexPosCommand() {
        return run(() -> intakeToReq(IntakeWristConstants.kStow)).until(() -> isCurrentMax() == true);
    }
    public Command intakePosCommand() {
        return run(() -> intakeToReq(IntakeWristConstants.kIntake)).withTimeout(2).until(() -> isCurrentMax() == true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //System.out.println(getPosition());
        // intakeToReq(reqPosition);
        current = turningMotor.getOutputCurrent();
        if (current > 0) {
            System.out.println(current);
        }
        SmartDashboard.putNumber("Pose", getPosition());
        SmartDashboard.putNumber("Current", current);
        
    }
}
