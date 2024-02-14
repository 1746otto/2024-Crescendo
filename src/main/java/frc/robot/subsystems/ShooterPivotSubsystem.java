package frc.robot.subsystems;


import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class ShooterPivotSubsystem {
    SparkPIDController m_pidController;

    CANSparkMax master;
    CANSparkMax slave;
    
    public ShooterPivotSubsystem(){
      master = new CANSparkMax(0,MotorType.kBrushless);
      slave = new CANSparkMax(1,MotorType.kBrushless);//Might need an inversion
      slave.follow(master);//Might need to have a workaround
      slave.setInverted(true);//Might need to change
      SparkAbsoluteEncoder encoder = master.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

      encoder.setAverageDepth(8); //To change 
      encoder.getPosition();
      m_pidController = master.getPIDController();

      
      master.getPIDController().setFeedbackDevice(encoder);
    }
    public void setRequest(double position){
        m_pidController.setReference(position, CANSparkMax.ControlType.kPosition);
    
    }
 
    public Command runPivot(){
        return new InstantCommand(() -> setRequest(Math.PI));
    }

    
}
    
  
