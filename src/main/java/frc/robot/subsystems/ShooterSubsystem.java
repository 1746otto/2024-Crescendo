// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterWristConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates new ShooterSubsystem. */
  
  /** CANSparkMax motor controller for the top shooting roller with PID control. */
  private TalonFX shooterRoller;
  
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
    shooterRoller = new TalonFX(ShooterConstants.kShooterTopRollerMotorID);
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterRoller.setInverted(true);

    shooterRoller.getConfigurator().apply(rollerConfig);

    //Setting PID values for the top shooting roller
    Slot0Configs pidController = rollerConfig.Slot0;
    pidController.kD = ShooterConstants.kV;
    
    // pidController = shooterRoller.getPIDController();
    // pidController.setP(0);
    // pidController.setI(0);
    // pidController.setD(0);
    // pidController.setFF(ShooterConstants.kV);
    
    

    // Making the bottom roller follow the top roller
    

    // Initialization of analog input for beam break detection
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 40;
    SmartDashboard.putNumber("Speed", targetVelocity);

  }

  /**
   * Sets the speed for the top shooting roller and returns a BooleanConsumer (placeholder for future functionality).
   */
  public void setOutput(double speed) {
    shooterRoller.set(speed);
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
    return setSpeedCommand(ShooterConstants.kShoot).andThen(StopCommand());
  }
  
  public Command ReverseCommand(){
    return setSpeedCommand(ShooterConstants.kReverse);
  }
  public Command StopCommand(){
    return setSpeedCommand(ShooterConstants.kStop);
  }

  public double getRPM() {
    return shooterRoller.getVelocity();
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
    pidController.setReference(RPM, ControlType.kVelocity, 0, (Math.round(RPM) == 0.0) ? 0 : Math.copySign(ShooterConstants.kS, RPM), ArbFFUnits.kVoltage);
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
    SmartDashboard.putNumber("shooterspeed", shooterRoller.getEncoder().getVelocity());  
  }

}