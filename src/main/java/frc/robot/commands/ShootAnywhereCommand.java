package frc.robot.commands;

import java.text.FieldPosition;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.constants.DynamicShootingConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

public class ShootAnywhereCommand extends Command {
  
  CommandSwerveDrivetrain swerve;
  Vision vision;
  ShooterSubsystem shooter;
  ShooterPivotSubsystem pivot;
  LEDSubsystem leds;
  ProfiledPIDController pidController;

  DoubleSupplier xAxisSupplier;
  DoubleSupplier yAxisSupplier;

  SwerveRequest.FieldCentricFacingAngle request;

  Translation2d speakerPose;

  ShootAnywhereCommand(CommandSwerveDrivetrain swerveSubsystem, Vision visionSubsystem, ShooterSubsystem shooterSubsystem, ShooterPivotSubsystem pivotSubsystem, LEDSubsystem ledSubsystem, DoubleSupplier xAxis, DoubleSupplier yAxis) {
    
    swerve = swerveSubsystem;
    vision = visionSubsystem;
    shooter = shooterSubsystem;
    pivot = pivotSubsystem;
    leds = ledSubsystem;
    xAxisSupplier = xAxis;
    yAxisSupplier = yAxis;
    
    pidController = new ProfiledPIDController(DynamicShootingConstants.kP, DynamicShootingConstants.kI, DynamicShootingConstants.kD, new Constraints(DynamicShootingConstants.kMaxAngularVelocity, DynamicShootingConstants.kMaxAngularAcceleration));
    request = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(6 * 0.1).withRotationalDeadband(1.5 * Math.PI * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    request.HeadingController = new PhoenixPIDController(DynamicShootingConstants.kP, DynamicShootingConstants.kI, DynamicShootingConstants.kD);
    speakerPose = (DriverStation.getAlliance().get() == Alliance.Blue) ? new Translation2d(FieldConstants.blueSpeakerX, FieldConstants.blueSpeakerY) : new Translation2d(FieldConstants.redSpeakerX, FieldConstants.redSpeakerY);
  }

  @Override
  public void initialize() {
    swerve.applyRequest(() -> request.withVelocityX(-xAxisSupplier.getAsDouble() * 6).withVelocityY(-yAxisSupplier.getAsDouble() * 6).withTargetDirection(swerve.getState().Pose.getTranslation().minus(speakerPose).getAngle()));
  }

  @Override
  public void execute() {

    // Find above and below keys
    double distance = swerve.getState().Pose.getTranslation().minus(speakerPose).getNorm();

    Map.Entry<Double, Pair<Double, Double>> lowEntry = DynamicShootingConstants.distanceMap.floorEntry(distance);
    Map.Entry<Double, Pair<Double, Double>> highEntry = DynamicShootingConstants.distanceMap.ceilingEntry(distance);
    
    // Find and apply interpolated angle and speed
    if (lowEntry.getKey() == null || highEntry.getKey() == null) {
      leds.setToHue(1);
      return;
    }
    else {
      leds.setToHue(65);
    }

    double interpolationValue = (distance - lowEntry.getKey())/(highEntry.getKey() - lowEntry.getKey());
    
    // Will get optimised away :)
    double shooterRPM = (highEntry.getValue().getFirst() - lowEntry.getValue().getFirst()) * interpolationValue + lowEntry.getValue().getFirst();
    double shooterAngle = (highEntry.getValue().getSecond() - lowEntry.getValue().getSecond()) * interpolationValue + lowEntry.getValue().getSecond();

    shooter.setRequest(shooterRPM);
    pivot.setRequest(shooterAngle);

  }

}
