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
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class ShooterPivotSubsystem extends SubsystemBase{
    private SparkPIDController m_pidController;
    private SparkAbsoluteEncoder encoder;

    private CANSparkMax master;
    private CANSparkMax slave;
  
    //Poses and tolerances
    public static double ampPos = 40;//to change
    public static double tolerance = 0.3;//To change
    private double targetPose;
    private double limit = 5.52380952383 / ( 2 * Math.PI);
    private static double kDt = 0.02;

    private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));//Need to tune and change
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    public ShooterPivotSubsystem(){
      master = new CANSparkMax(0,MotorType.kBrushless);
      slave = new CANSparkMax(1,MotorType.kBrushless);
      slave.follow(master);//Might need to have a workaround
      slave.setInverted(true);//Might need to change
      encoder = master.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

      encoder.setAverageDepth(8); //To change 
      m_pidController = master.getPIDController();

      master.getPIDController().setFeedbackDevice(encoder);
      double max = encoder.getPosition() + limit;
      double min = encoder.getPosition() - limit;
      
      master.setSoftLimit(SoftLimitDirection.kForward, (float) max);
      master.setSoftLimit(SoftLimitDirection.kReverse, (float) min);
      
    }

    public void setRequest(double position) {
        targetPose = position;
        m_pidController.setReference(m_setpoint.position, CANSparkMax.ControlType.kPosition);
    }
    public boolean atRequest(double position) {
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
        m_setpoint = m_profile.calculate(kDt,m_setpoint,m_goal);
    }    
}
    
  
