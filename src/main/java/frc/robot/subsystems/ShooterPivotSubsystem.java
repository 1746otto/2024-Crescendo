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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterWristConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;



public class ShooterPivotSubsystem extends SubsystemBase{
    private SparkPIDController m_pidController;
    private SparkAbsoluteEncoder encoder;

    public CANSparkMax master;
    private CANSparkMax slave;

    private double targetPose = ShooterWristConstants.kIntakePos;
    private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));//Need to tune and change

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_temporaryState = new TrapezoidProfile.State();

    private double lastTime = 0;
    private double deltaTime = 0;
    private double startTime = 0;

    public ShooterPivotSubsystem() {

      master = new CANSparkMax(ShooterWristConstants.kShooterMasterID,MotorType.kBrushless);
      slave = new CANSparkMax(ShooterWristConstants.kShooterSlaveID,MotorType.kBrushless);
      slave.follow(master, true);//Might need to have a workaround//Might need to change
      encoder = master.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

      master.setIdleMode(IdleMode.kBrake);
      slave.setIdleMode(IdleMode.kBrake);
      master.setSmartCurrentLimit(50);
      slave.setSmartCurrentLimit(50);
      m_pidController = master.getPIDController();
      m_pidController.setFeedbackDevice(encoder);
      m_pidController.setP(12.8); // 6.4
      m_pidController.setD(5.0);

      double max = ShooterWristConstants.kIntakePos + ShooterWristConstants.kLimit;//Might need to be changed to be through sparkmax
      double min = ShooterWristConstants.kIntakePos - ShooterWristConstants.kLimit;
      
      master.setSoftLimit(SoftLimitDirection.kForward, (float) max);
      master.setSoftLimit(SoftLimitDirection.kReverse, (float) min);
      
    }
    public void test() {
        master.set(0.1);
    }

    public void setRequest(double position) {
        m_goal = new TrapezoidProfile.State(position, 0); //Skeptical about this
        m_setpoint = new TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity());
        m_profile.calculate(0, m_setpoint, m_goal);
        startTime = Timer.getFPGATimestamp();

    }
    public boolean atPosition(double position) {
        return (Math.abs(encoder.getPosition() - position) < ShooterWristConstants.kTolerance);
    }

    public boolean atSetpoint() {
        return (Math.abs(encoder.getPosition() - m_goal.position) < ShooterWristConstants.kTolerance); 
    }

    public double getTargetPose(){
        return m_goal.position;
    }

    public void stop() {
        master.set(0);
    }

    public Command runPivot(double position) {
        return run(() -> setRequest(position)).until(() -> atPosition(position));
    }

    public Command goToAmpPose(){
        return runPivot(ShooterWristConstants.kAmpPos);
    }

    public Command goToNormalPos() {
        return runPivot(ShooterWristConstants.kIntakePos);
    }
    public Command goToPodiumPos() {
        return runPivot(ShooterWristConstants.kPodiumPos);
    }
    
    public Command stopCommand() {
        return new InstantCommand(() -> stop());
    }
    public Command testShooter() {
        return run(() -> test()).finallyDo(() -> master.set(0));
    }
    @Override
    public void periodic() {
        deltaTime = Timer.getFPGATimestamp() - lastTime;
        SmartDashboard.putNumber("TargetPose", m_goal.position);
        SmartDashboard.putNumber("CurrentPose", encoder.getPosition());
        targetPose = SmartDashboard.getNumber("Setpoint", m_goal.position);
        if (targetPose != m_goal.position) {
            setRequest(targetPose);
        }
        //m_setpoint = m_profile.calculate(kDt,m_setpoint,m_goal);
        //m_pidController.setReference(m_setpoint.position, CANSparkMax.ControlType.kPosition);
        
        m_setpoint = m_profile.calculate(deltaTime, m_setpoint, m_goal);
        
        if (m_profile.isFinished(Timer.getFPGATimestamp() - startTime)) {
            master.getPIDController().setReference(m_setpoint.position, ControlType.kPosition, 0, ShooterWristConstants.kG * Math.sin(encoder.getPosition()*2*Math.PI)+ Math.copySign(ShooterWristConstants.kS, m_temporaryState.velocity), ArbFFUnits.kVoltage);
        }
        else if (Math.abs(encoder.getPosition() - m_goal.position) > ShooterWristConstants.kTolerance){
            master.getPIDController().setReference(m_goal.position, ControlType.kPosition, 0, ShooterWristConstants.kG * Math.sin(encoder.getPosition()*2*Math.PI)+ Math.copySign(ShooterWristConstants.kS, m_temporaryState.velocity), ArbFFUnits.kVoltage);
        }
        else {
            master.set(0);
        }
    }    
}
    
  
