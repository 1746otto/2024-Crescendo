package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;


public class IntakeSubsystem extends SubsystemBase{
    CANSparkMax turningMotor;
    CANSparkMax intakeMotor;
    SparkPIDController pidController;
    boolean outside;
    double reqPosition;

    public IntakeSubsystem(){
        turningMotor = new CANSparkMax(IntakeConstants.kIntakeTurnID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeID, MotorType.kBrushless);
        pidController = turningMotor.getPIDController();
        pidController.setP(IntakeConstants.kP);
        reqPosition = IntakeConstants.originPosition;
    }

    public void intake(){
        intakeMotor.set(IntakeConstants.kIntakeSpeed);
    }
    public void outtake(){
        intakeMotor.set(-IntakeConstants.kIntakeSpeed);
    }
    public void angleIn(){
        pidController.setReference(IntakeConstants.originPosition, ControlType.kPosition);
    }
    public void angleOut(){
        pidController.setReference(IntakeConstants.outPosition, ControlType.kPosition);
    }
    


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }
}
