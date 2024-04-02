package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.BackpackWristConstants;
import frc.robot.Constants.IntakeWristConstants;

public class BackpackWristSubsystem extends SubsystemBase{
    
    CANSparkMax wristMotor;
    private SparkPIDController m_pidController;
    private SparkAbsoluteEncoder encoder;

    double targetPose;

    public BackpackWristSubsystem() {

        // Initialization of motor controllers and PID controller
        wristMotor = new CANSparkMax(BackpackWristConstants.kMotorID, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake); // Might Need to change
        encoder = wristMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        
        m_pidController = wristMotor.getPIDController();
        m_pidController.setFeedbackDevice(encoder);
        m_pidController.setP(BackpackWristConstants.kP);
        m_pidController.setI(BackpackWristConstants.kI);
        m_pidController.setD(BackpackWristConstants.kD);
        m_pidController.setFF(BackpackWristConstants.kFF);

        wristMotor.setSmartCurrentLimit(BackpackWristConstants.kSupplyLimit);

    }

    public void stop() {
        wristMotor.set(0);
    }
    
    public void setRequest(double position) {
        targetPose = position;
    }
    
    public boolean atPosition(double position) {
        return (Math.abs(encoder.getPosition() - position) < BackpackWristConstants.kTolerance);
    }

    public boolean atSetpoint() {
        return (Math.abs(encoder.getPosition() - targetPose) < BackpackWristConstants.kTolerance); 
    }

    public Command runWrist(double position) {
        return runOnce(() -> setRequest(position)).andThen(new WaitUntilCommand(() -> isAtReqPosition(position)));
    }

    public double getPosition() {
        return encoder.getPosition();
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

    public Command intakePosCommand() {
        return runWrist(BackpackWristConstants.kIntake);
    }
    public Command stowPosCommand() {
        return runWrist(BackpackWristConstants.kStow);
    }

    public Command stopMotorCommand(){
        return runOnce(this::stop);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //System.out.println(getPosition());
        SmartDashboard.putNumber("currentPoseBackpack", getPosition());
        SmartDashboard.putNumber("targetPoseBackpack", targetPose);
        
    }
}
