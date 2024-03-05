// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterWristConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates new ShooterSubsystem. */
  
  /** CANSparkMax motor controller for the top shooting roller with PID control. */
  private CANSparkMax topRollerNeo;
  
  /** CANSparkMax motor controller for the bottom shooting roller (follows top roller). */
  private CANSparkMax bottomRollerNeo;
  
  /** Analog input for detecting beam breaks. */
  private AnalogInput beamBreak;
  
  /** PID controller for the top shooting roller. */
  private SparkPIDController pidController;

  /** Speed settings for various roller movements. */
  private double topShootingSpeed = frc.robot.Constants.ShooterConstants.kShoot;

  /** Supplier for maintaining the state of the beam break. */
  private BooleanSupplier beamBreakLastState;
  
  private double setpoint = 0;
  private double temp = 0; // Use in various intermediary steps.

  /**
   * Creates a new ShooterSubsystem with initialized motor controllers and other necessary components.
   */
  public ShooterSubsystem() {
    // Initialization of motor controllers
    topRollerNeo = new CANSparkMax(ShooterConstants.kShooterTopRollerMotorID, MotorType.kBrushless);
    bottomRollerNeo = new CANSparkMax(ShooterConstants.kShooterBottomRollerMotorID, MotorType.kBrushless);
  
    // Making the bottom roller follow the top roller
    //bottomRollerNeo.follow(topRollerNeo, true);

    topRollerNeo.getEncoder().setAverageDepth(2);
    topRollerNeo.getEncoder().setMeasurementPeriod(16);
    
    // Likely unneccessary
    bottomRollerNeo.getEncoder().setAverageDepth(2);
    bottomRollerNeo.getEncoder().setMeasurementPeriod(16);

    topRollerNeo.enableVoltageCompensation(12);
    topRollerNeo.enableVoltageCompensation(12);

    topRollerNeo.setIdleMode(IdleMode.kCoast);
    bottomRollerNeo.setIdleMode(IdleMode.kCoast);

    topRollerNeo.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
    topRollerNeo.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);

    //Setting PID values for the top shooting roller
    pidController = topRollerNeo.getPIDController();
    pidController.setP(ShooterConstants.kP);
    pidController.setI(ShooterConstants.kI);
    pidController.setD(ShooterConstants.kD);
    pidController.setFF(ShooterConstants.kV);

    

    
  }

  /**
   * Sets the speed for the top shooting roller and returns a BooleanConsumer (placeholder for future functionality).
   */
  public void setSpeed(double speed) {
    topRollerNeo.set(speed);
  }

  /**
   * Returns a BooleanSupplier representing the state of the beam break.
   */
  public BooleanSupplier isBeamBreakBroken() {
    return beamBreakLastState;
  }

  public void setRequest(double RPM) {
    setpoint = RPM;
    pidController.setReference(RPM, ControlType.kVelocity, 0, Math.copySign(ShooterConstants.kS, RPM), ArbFFUnits.kVoltage);
  }

  /**
   * Creates a command for shooting based on certain conditions.
   */
  public Command ShootCommand(){
    return setSpeedCommand(ShooterConstants.kShoot);
  }
  
  public Command ReverseCommand(){
    return setSpeedCommand(ShooterConstants.kReverse);
  }
  public Command StopCommand(){
    return setSpeedCommand(ShooterConstants.kStop);
  }

  public Command setSpeedCommand(double speed) {
    return run(() -> setRequest(speed));
  }

  /**
   * Periodic method for updating the state of the beam break.
   */
  public void periodic() {
    SmartDashboard.putNumber("Velocity", topRollerNeo.getEncoder().getVelocity());
    temp = SmartDashboard.getNumber("Request velocity", setpoint);
   

  }

}