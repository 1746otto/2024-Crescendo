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
  private CANSparkMax indexerNeo;
  private CANSparkMax primerNeo;
  private CANSparkMax topRollerNeo;
  private CANSparkMax bottomRollerNeo;
  private AnalogInput beamBreak;

  private SparkPIDController pidController;

  private double topShootingSpeed = frc.robot.Constants.ShooterConstants.kTopShootingRollerSpeed;
  private double bottomShootingSpeed = frc.robot.Constants.ShooterConstants.kBottomShootingRollerSpeed;
  private double primingSpeed = frc.robot.Constants.ShooterConstants.kPrimingRollerSpeed;
  private double indexingSpeed = frc.robot.Constants.ShooterConstants.kIndexingRollerSpeed;

  private BooleanSupplier beamBreakLastState;


  public ShooterSubsystem() {
    indexerNeo = new CANSparkMax(ShooterConstants.kIndexingRollerMotorID, MotorType.kBrushless);

    primerNeo = new CANSparkMax(ShooterConstants.kPrimingRollerMotorID, MotorType.kBrushless);

    topRollerNeo = new CANSparkMax(ShooterConstants.kShootingTopRollerMotorID, MotorType.kBrushless);
    pidController = topRollerNeo.getPIDController();
    pidController.setP(ShooterConstants.topRollerKP);
    pidController.setI(ShooterConstants.topRollerKI);
    pidController.setD(ShooterConstants.topRollerKD);

    bottomRollerNeo = new CANSparkMax(ShooterConstants.kShootingBottomRollerMotorID, MotorType.kBrushless);
    bottomRollerNeo.follow(topRollerNeo);

    beamBreak = new AnalogInput(ShooterConstants.kShooterAnalogInputChannel);    
  }

  public void primeNote() {
    primerNeo.set(primingSpeed);
  }

  public void indexNote() {
    indexerNeo.set(indexingSpeed);
  }

  public BooleanConsumer runTopRollers() {
    topRollerNeo.set(topShootingSpeed);
    return null;
  }

  public void runBottomRollers() {
    bottomRollerNeo.set(bottomShootingSpeed);
  }

  public BooleanSupplier isBeamBreakBroken() {
    return beamBreakLastState;
  }

  public void primeAndIndexNote() {
    primerNeo.set(primingSpeed);
    indexerNeo.set(indexingSpeed);
  }

  public void stop() {
    primerNeo.set(0);
    indexerNeo.set(0);
    topRollerNeo.set(0);
  }

  public Command shootCommand() {
    return run (this::primeAndIndexNote).until(beamBreakLastState).finallyDo(runTopRollers()).andThen(() -> stop());
  }

  public void periodic() {
    beamBreakLastState = () -> ((Math.floor(beamBreak.getVoltage()) > 0) && (primerNeo.get() < 0));
  }

}