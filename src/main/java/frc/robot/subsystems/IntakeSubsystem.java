package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        reqPosition = IntakeConstants.kOriginPosition;
    }

    public void intake(){
        intakeMotor.set(IntakeConstants.kIntakeSpeed);
    }
    public void outtake(){
        intakeMotor.set(-IntakeConstants.kIntakeSpeed);
    }
    public void intakeToReq(double req){
        pidController.setReference(req, ControlType.kPosition);
    }
    public void setRequest(double req){
        reqPosition = req;
    }
    public double getPosition(){
        return turningMotor.getEncoder().getPosition();
    }
    public boolean isAtReqPosition(double reqPos){
        if ((getPosition() >= (reqPos - IntakeConstants.kTolerance))
         && (getPosition() <= (reqPos + IntakeConstants.kTolerance))){
            return true;
        }
        return false;
    }

    //COMMANDS
    public InstantCommand IntakeCommand(){
        return new InstantCommand(() -> intake());
    }
    public InstantCommand OuttakeCommand(){
        return new InstantCommand(() -> outtake());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        intakeToReq(reqPosition);
    }
}