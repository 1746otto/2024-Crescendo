package frc.robot.subsystems;


import java.util.function.BooleanSupplier;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class ShooterPivotSubsystem extends SubsystemBase{
    SparkPIDController m_pidController;
    SparkAbsoluteEncoder encoder;

    CANSparkMax master;
    CANSparkMax slave;

    //Poses
    public static double ampPos = 40;//to change
    double tolerance = 5;
    double targetPose;

    public ShooterPivotSubsystem(){
      master = new CANSparkMax(0,MotorType.kBrushless);
      slave = new CANSparkMax(1,MotorType.kBrushless);
      slave.follow(master);//Might need to have a workaround
      slave.setInverted(true);//Might need to change
      encoder = master.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

      encoder.setAverageDepth(8); //To change 
      encoder.getPosition();
      m_pidController = master.getPIDController();

      master.getPIDController().setFeedbackDevice(encoder);
    }
    public void setRequest(double position) {
        m_pidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }
    public boolean atRequest(double position) {
        targetPose = position;
        return (Math.abs(encoder.getPosition() - position) < tolerance);
    }
    public double getTargetPose(){
        return targetPose;
    }
    public void stop() {
        master.set(0);
    }
    public Command runPivot(double position) {
        return new RunCommand(() -> setRequest(position)).until(() -> atRequest(position));
    }
    public Command goToAmpPose(){
        return runPivot(ampPos);
    }
    public Command stopCommand() {
        return new InstantCommand(() -> stop());
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("TargetPose", targetPose);
        SmartDashboard.putNumber("CurrentPose", encoder.getPosition());
    }    
}
    
  
