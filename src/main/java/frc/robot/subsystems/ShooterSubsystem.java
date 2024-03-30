// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates new ShooterSubsystem. */
  
  private TalonFX shooterLeader;
  private TalonFX shooterFollower;
  
  /** Analog input for detecting beam breaks. */
  private AnalogInput beamBreak;
  
  /** PID controller for the top shooting roller. */
  private SparkPIDController pidController;

  /** Speed settings for various roller movements. */
  private double topShootingSpeed = frc.robot.Constants.ShooterConstants.kShoot;

  /** Supplier for maintaining the state of the beam break. */
  private BooleanSupplier beamBreakLastState;

  private double targetVelocity;

  /**
   * Creates a new ShooterSubsystem with initialized motor controllers and other necessary components.
   */
  public ShooterSubsystem() {
    // Initialization of motor controllers
    shooterLeader = new TalonFX(ShooterConstants.kShooterTopRollerMotorID);
    shooterFollower = new TalonFX(ShooterConstants.kShooterBottomRollerMotorID);
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    Slot0Configs pidController = rollerConfig.Slot0;
    pidController.kP = ShooterConstants.kP;
    pidController.kI = ShooterConstants.kI;
    pidController.kD = ShooterConstants.kD;
    pidController.kS = ShooterConstants.kS;
    pidController.kV = ShooterConstants.kV;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
    shooterLeader.getConfigurator().apply(rollerConfig);
    shooterFollower.setControl(new Follower(ShooterConstants.kShooterTopRollerMotorID, true));

    //Setting PID values for the top shooting roller not how this works
   
    
    

    // pidController = shooterRoller.getPIDController();
    // pidController.setP(0);
    // pidController.setI(0);
    // pidController.setD(0);
    // pidController.setFF(ShooterConstants.kV);
    
    

    // Making the bottom roller follow the top roller
    

    // Initialization of analog input for beam break detection
    // Also in the wrong spot but i have to commit.
    
    SmartDashboard.putNumber("Speed", targetVelocity);

  }

  /**
   * Sets the speed for the top shooting roller and returns a BooleanConsumer (placeholder for future functionality).
   */
  public void setOutput(double speed) {
    shooterLeader.set(speed);
  }

  /**
   * Returns a BooleanSupplier representing the state of the beam break.
   */ 
  public BooleanSupplier isBeamBreakBroken() {
    return beamBreakLastState;
  }

  /**
   * Creates a command for shooting based on certain conditions.
   */
  public Command ShootCommand(){
    return setSpeedCommand(ShooterConstants.kShoot);
  }
  public Command tempShooter(){
    return run(() -> shooterLeader.set(0.5));
  }
  public Command tempStop(){
    return run(() -> shooterLeader.set(0));
  }
  
  public Command ReverseCommand(){
    return setSpeedCommand(ShooterConstants.kReverse);
  }
  public Command StopCommand(){
    return setSpeedCommand(ShooterConstants.kStop);
  }

  public double getRPM() {
    return shooterLeader.getVelocity().getValueAsDouble()*60;
  }

  public void stop() {
    setRequest(0);
  }

  public boolean isAtReq() {
    return Math.abs(targetVelocity - getRPM()) < ShooterConstants.kTolerance;
  }

  public Command setSpeedCommand(double speed) {
    return runOnce(() -> setRequest(speed));
  }

  public void setRequest(double RPM) {
    targetVelocity = RPM;
    shooterLeader.setControl(new VelocityVoltage(RPM/60));
    // shooterLeader.setControl(new VelocityVoltage(RPM/60, 0, false, 0, 0, false, false, false));
    //pidController.setReference(RPM, ControlType.kVelocity, 0, (Math.round(RPM) == 0.0) ? 0 : Math.copySign(ShooterConstants.kS, RPM), ArbFFUnits.kVoltage);
  }

  public Command setRequestCommand(double RPM) {
    return runOnce(() -> setRequest(RPM));
  }

  /**
   * Periodic method for updating the state of the beam break.
   */
  public void periodic() {
    // if (SmartDashboard.getNumber("Speed", targetVelocity) != targetVelocity) {
    //   targetVelocity = SmartDashboard.getNumber("Speed", targetVelocity);
    // }
    //pidController.setReference(targetVelocity, ControlType.kVelocity, 0, ShooterConstants.kS, ArbFFUnits.kVoltage);
    SmartDashboard.putNumber("shooterspeed", getRPM());  
  }

}