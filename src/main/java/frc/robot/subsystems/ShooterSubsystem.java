// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Class for ShooterSubystem to propel game pieces from holding space into scoring stations.
 */
public class ShooterSubsystem extends SubsystemBase {
  /** CANSparkMax motor controller for the top shooting roller with PID control. */
  private CANSparkMax topRollerNeo;
  
  /** CANSparkMax motor controller for the bottom shooting roller (follows top roller). */
  private CANSparkMax bottomRollerNeo;
  
  /** Analog input for detecting beam breaks. */
  private AnalogInput beamBreak;
  
  /** PID controller for the top shooting roller. */
  private SparkPIDController pidController;

  /** Supplier for maintaining the state of the beam break. */
  private BooleanSupplier beamBreakLastState;

  /**
   * Creates a new ShooterSubsystem with initialized motor controllers and other necessary components.
   */
  public ShooterSubsystem() {
    // Initialization of motor controllers
    topRollerNeo = new CANSparkMax(ShooterConstants.kShooterTopRollerMotorID, MotorType.kBrushless);
    //topRollerNeo.setInverted(true);
    

    // Setting PID values for the top shooting roller
    pidController = topRollerNeo.getPIDController();
    pidController.setP(ShooterConstants.topRollerKP);
    pidController.setI(ShooterConstants.topRollerKI);
    pidController.setD(ShooterConstants.topRollerKD);

    // Making the bottom roller follow the top roller
    bottomRollerNeo = new CANSparkMax(ShooterConstants.kShooterBottomRollerMotorID, MotorType.kBrushless);
    bottomRollerNeo.follow(topRollerNeo);
    //bottomRollerNeo.setInverted(false);

    // Initialization of analog input for beam break detection
    beamBreak = new AnalogInput(ShooterConstants.kShooterAnalogInputChannel);    

  }

  /**
   * Sets the speed for the top shooting roller and returns a BooleanConsumer (placeholder for future functionality).
   */
  public void runShooterRollers(double speed) {
    topRollerNeo.set(speed);
  }

  /**
   * Returns a BooleanSupplier representing if the beambreak has been broken or not.
   */
  public BooleanSupplier isBeamBreakBroken() {
    return beamBreakLastState;
  }




  /**
   * Command for running the shooter
   * @return
   */
  public Command shootCommand(){
    return run(() -> runShooterRollers(ShooterConstants.kShooterRollerSpeed));
  }
  /**
   * Command for running the shooter backwards if game piece is sticking out too much.
   * @return
   */
  public Command reverseCommand(){
    return run(() -> runShooterRollers(-ShooterConstants.kShooterRollerSpeed));
  }
  /**
   * Command to stop the shooter.
   * @return
   */
  public Command stopCommand(){
    return run(() -> runShooterRollers(ShooterConstants.kShooterStopSpeed));
  }

  /**
   * Periodic method for updating the state of the beam break.
   */
  public void periodic() {
    beamBreakLastState = () -> ((Math.floor(beamBreak.getVoltage()) > 0));
  }

}