// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates new ShooterSubsystem. */

  /** CANSparkMax motor controller for the indexing roller. */
  private CANSparkMax indexerNeo;

  /** CANSparkMax motor controller for the priming roller. */
  private CANSparkMax primerNeo;
  
  /** CANSparkMax motor controller for the top shooting roller with PID control. */
  private CANSparkMax topRollerNeo;
  
  /** CANSparkMax motor controller for the bottom shooting roller (follows top roller). */
  private CANSparkMax bottomRollerNeo;
  
  /** Analog input for detecting beam breaks. */
  private AnalogInput beamBreak;
  
  /** PID controller for the top shooting roller. */
  private SparkPIDController pidController;

  /** Speed settings for various roller movements. */
  private double topShootingSpeed = frc.robot.Constants.ShooterConstants.kTopShootingRollerSpeed;
  private double bottomShootingSpeed = frc.robot.Constants.ShooterConstants.kBottomShootingRollerSpeed;
  private double primingSpeed = frc.robot.Constants.ShooterConstants.kPrimingRollerSpeed;
  private double indexingSpeed = frc.robot.Constants.ShooterConstants.kIndexingRollerSpeed;

  /** Supplier for maintaining the state of the beam break. */
  private BooleanSupplier beamBreakLastState;

  /**
   * Creates a new ShooterSubsystem with initialized motor controllers and other necessary components.
   */
  public ShooterSubsystem() {
    // Initialization of motor controllers
    indexerNeo = new CANSparkMax(ShooterConstants.kIndexingRollerMotorID, MotorType.kBrushless);
    primerNeo = new CANSparkMax(ShooterConstants.kPrimingRollerMotorID, MotorType.kBrushless);
    topRollerNeo = new CANSparkMax(ShooterConstants.kShootingTopRollerMotorID, MotorType.kBrushless);

    // Setting PID values for the top shooting roller
    pidController = topRollerNeo.getPIDController();
    pidController.setP(ShooterConstants.topRollerKP);
    pidController.setI(ShooterConstants.topRollerKI);
    pidController.setD(ShooterConstants.topRollerKD);

    // Making the bottom roller follow the top roller
    bottomRollerNeo = new CANSparkMax(ShooterConstants.kShootingBottomRollerMotorID, MotorType.kBrushless);
    bottomRollerNeo.follow(topRollerNeo);

    // Initialization of analog input for beam break detection
    beamBreak = new AnalogInput(ShooterConstants.kShooterAnalogInputChannel);    

  }
  /**
   * Sets the priming roller speed for priming the shooting mechanism.
   */
  public void primeNote() {
    primerNeo.set(primingSpeed);
  }

  /**
   * Sets the indexing roller speed for indexing notes in the shooting mechanism.
   */
  public void indexNote() {
    indexerNeo.set(indexingSpeed);
  }

  /**
   * Sets the speed for the top shooting roller and returns a BooleanConsumer (placeholder for future functionality).
   */
  public BooleanConsumer runTopRollers() {
    topRollerNeo.set(topShootingSpeed);
    return null;
  }

  /**
   * Sets the speed for the bottom shooting roller.
   */
  public void runBottomRollers() {
    bottomRollerNeo.set(bottomShootingSpeed);
  }

  /**
   * Returns a BooleanSupplier representing the state of the beam break.
   */
  public BooleanSupplier isBeamBreakBroken() {
    return beamBreakLastState;
  }

  /**
   * Sets the speed for both the priming and indexing rollers.
   */
  public void primeAndIndexNote() {
    primerNeo.set(primingSpeed);
    indexerNeo.set(indexingSpeed);
  }

  /**
   * Stops all motor movements.
   */
  public void stop() {
    primerNeo.set(0);
    indexerNeo.set(0);
    topRollerNeo.set(0);
  }

  /**
   * Creates a command for shooting based on certain conditions.
   */
  public Command shootCommand() {
    return run (this::primeAndIndexNote).until(beamBreakLastState).finallyDo(runTopRollers()).andThen(() -> stop());
  }

  /**
   * Periodic method for updating the state of the beam break.
   */
  public void periodic() {
    beamBreakLastState = () -> ((Math.floor(beamBreak.getVoltage()) > 0) && (primerNeo.get() < 0));
  }

}