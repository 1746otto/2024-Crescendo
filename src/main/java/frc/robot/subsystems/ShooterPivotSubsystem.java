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
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterWristConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class ShooterPivotSubsystem extends SubsystemBase{
    private SparkPIDController m_pidController;
    private SparkAbsoluteEncoder encoder;

    public CANSparkMax master;
    private CANSparkMax slave;
  
    //Poses and tolerances
    public static double tolerance = Math.toRadians(10) / ( 2 * Math.PI );//To change
    private double limit = 0.5 / ( 2 * Math.PI);
    private static double kDt = 0.02;

    private double targetPose = ShooterWristConstants.kintakePos;
    private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));//Need to tune and change
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private double currentTime;

    public ShooterPivotSubsystem(){
      master = new CANSparkMax(ShooterWristConstants.ShooterMasterID,MotorType.kBrushless);
      slave = new CANSparkMax(ShooterWristConstants.ShooterSlaveID,MotorType.kBrushless);
      slave.follow(master, true);//Might need to have a workaround//Might need to change
      encoder = master.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

      master.setIdleMode(IdleMode.kBrake);
      slave.setIdleMode(IdleMode.kBrake);
      master.setSmartCurrentLimit(50);
      m_pidController = master.getPIDController();
      encoder.setAverageDepth(8); //To change 
      m_pidController.setFeedbackDevice(encoder);
      m_pidController.setP(12.8);//6.4
      m_pidController.setD(5.0);

      double max = ShooterWristConstants.kintakePos + ShooterWristConstants.klimit;//Might need to be changed to be through sparkmax
      double min = ShooterWristConstants.kintakePos - ShooterWristConstants.klimit;
      
      master.setSoftLimit(SoftLimitDirection.kForward, (float) max);
      master.setSoftLimit(SoftLimitDirection.kReverse, (float) min);
      
    }
    public void test() {
        master.set(0.1);
    }

    public void setRequest(double position) {
       m_goal = new TrapezoidProfile.State(position, ShooterConstants.shooterPivotVelocity); //Skeptical about this
       targetPose = position;
       currentTime = 0;

    }
    public boolean atRequest(double position) {
        return (Math.abs(encoder.getPosition() - position) < ShooterWristConstants.ktolerance);
    }

    public double getTargetPose(){
        return targetPose;
    }

    public void stop() {
        master.set(0);
    }

    public Command runPivot(double position) {
        return run(() -> setRequest(position)).until(() -> atRequest(position));
    }

    public Command goToAmpPose(){
        return runPivot(ShooterWristConstants.kampPos);
    }

    public Command goToNormalPos() {
        return runPivot(ShooterWristConstants.kintakePos);
    }
    public Command goToPodiumPos() {
        return runPivot(ShooterWristConstants.kpodiumPos);
    }
    
    public Command stopCommand() {
        return new InstantCommand(() -> stop());
    }
    public Command testShooter() {
        return run(() -> test()).finallyDo(() -> master.set(0));
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("TargetPose", targetPose);
        SmartDashboard.putNumber("CurrentPose", encoder.getPosition());

        m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

        if (m_profile.isFinished(currentTime))
        {
            if (Math.abs(master.getEncoder().getPosition() - m_setpoint.position) <= ShooterWristConstants.ktolerance)
            {
                master.stopMotor();
            }else
            {
                m_pidController.setReference(m_goal.position, CANSparkMax.ControlType.kPosition);
            }
        }

        currentTime += kDt;

        // wait until traj finished, then check if we are inside/outside tolerances,
        // if outside, set target to goal.position
        // if inside, stop motors
        
    }    
}
    
  
