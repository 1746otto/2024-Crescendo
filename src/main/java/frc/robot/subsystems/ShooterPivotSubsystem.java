package frc.robot.subsystems;


import java.util.function.BooleanSupplier;

import javax.management.openmbean.TabularType;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
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
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterWristConstants;

import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;



public class ShooterPivotSubsystem extends SubsystemBase{
    private SparkPIDController m_pidController;
    private Canandcoder encoder;

    public TalonFX master;
    private TalonFX slave;

    private double targetPose = ShooterWristConstants.kIntakePos;
    private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));//Need to tune and change

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_current = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_temporaryState = new TrapezoidProfile.State();

    private double lastTime = 0;
    private double deltaTime = 0;
    private double startTime = 0;

    public ShooterPivotSubsystem() {
      TalonFXConfiguration configs = new TalonFXConfiguration();
      master = new TalonFX(ShooterWristConstants.kShooterMasterID);
      slave = new TalonFX(ShooterWristConstants.kShooterSlaveID);
      slave.setControl(new Follower(ShooterWristConstants.kShooterMasterID, true));
      encoder = new Canandcoder(0);//Change
      configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      configs.CurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(50);
      master.getConfigurator().apply(configs);
      slave.getConfigurator().apply(configs);
      Slot0Configs pidController = configs.Slot0;
      pidController.kP = ShooterWristConstants.kP;
      pidController.kD = ShooterWristConstants.kD;
      pidController.kS = ShooterWristConstants.kS;
      //master.enableVoltageCompensation(12);
      //slave.enableVoltageCompensation(12);

      double max = ShooterWristConstants.kIntakePos + ShooterWristConstants.kLimit;//Might need to be changed to be through sparkmax
      double min = ShooterWristConstants.kIntakePos - ShooterWristConstants.kLimit;
    
      SmartDashboard.putNumber("target", targetPose);
      
      setRequest(encoder.getPosition());
    }
    public void test() {
        master.set(0.1);
    }

    public void setRequest(double position) {
        targetPose = position;
       

    }
    public boolean atPosition(double position) {
        return (Math.abs(encoder.getPosition() - position) < ShooterWristConstants.kTolerance);
    }

    public boolean atSetpoint() {
        return (Math.abs(encoder.getPosition() - targetPose) < ShooterWristConstants.kTolerance); 
    }

    public double getTargetPose(){
        return targetPose;
    }

    public void stop() {
        master.set(0);
    }

    public Command runPivot(double position) {
        return runOnce(() -> setRequest(position)).andThen(new WaitUntilCommand(() -> atPosition(position)));
    }

    public Command goToAmpPose(){
        return runPivot(ShooterWristConstants.kAmpPos);
    }

    /**
     * Sets the targetPose variable to indexing position and the waits until pivot is at that position.
     * 
     * <p> Ends on: pivot within tolerance
     * 
     * <p> End behavior: The pivot stop if it is within tolerance,
     * otherwise the pid will run regardless of whether or not the command is still running.
     * The pid will renable without a new command if the pivot falls out of tolerance.
     * @return SequentialCommandGroup
     */
    public Command goToIntakePos() {
        return runPivot(ShooterWristConstants.kIntakePos);
    }
    public Command goToPodiumPos() {
        return runPivot(ShooterWristConstants.kPodiumPos);
    }
    public Command goToSubCommand() {
        return runPivot(ShooterWristConstants.kSubwooferPos);
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
 
        if (!atSetpoint()){
            m_pidController.setReference(targetPose, ControlType.kPosition, 0, ShooterWristConstants.kG * Math.sin(encoder.getPosition()*2*Math.PI)+ Math.copySign(ShooterWristConstants.kS, targetPose - encoder.getPosition()), ArbFFUnits.kVoltage);
        }
        else {
            master.set(0);
        }
    }    
}
    
  
