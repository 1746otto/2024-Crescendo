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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Spline;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.spline.Spline.ControlVector;
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
  CubicHermiteSpline[] pivotSplines;
  CubicHermiteSpline[] shooterSplines;

  DoubleSupplier xAxisSupplier;
  DoubleSupplier yAxisSupplier;

  SwerveRequest.FieldCentricFacingAngle request;

  Translation2d[] pivotPositions = new Translation2d[DynamicShootingConstants.distanceMapLength - 2];
  Translation2d[] shooterSpeeds = new Translation2d[DynamicShootingConstants.distanceMapLength - 2];

  Translation2d speakerPose;

  double direction;

  public ShootAnywhereCommand(CommandSwerveDrivetrain swerveSubsystem, Vision visionSubsystem, ShooterSubsystem shooterSubsystem, ShooterPivotSubsystem pivotSubsystem, LEDSubsystem ledSubsystem, DoubleSupplier xAxis, DoubleSupplier yAxis, double direction) {
    
    swerve = swerveSubsystem;
    vision = visionSubsystem;
    shooter = shooterSubsystem;
    pivot = pivotSubsystem;
    leds = ledSubsystem;
    xAxisSupplier = xAxis;
    yAxisSupplier = yAxis;
    
    this.direction = direction;

    pidController = new ProfiledPIDController(DynamicShootingConstants.kP, DynamicShootingConstants.kI, DynamicShootingConstants.kD, new Constraints(DynamicShootingConstants.kMaxAngularVelocity, DynamicShootingConstants.kMaxAngularAcceleration));
    request = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(6 * 0.1).withRotationalDeadband(1.5 * Math.PI * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    request.HeadingController = new PhoenixPIDController(DynamicShootingConstants.kP, DynamicShootingConstants.kI, DynamicShootingConstants.kD);
    speakerPose = (DriverStation.getAlliance().get() == Alliance.Blue) ? new Translation2d(FieldConstants.blueSpeakerX, FieldConstants.blueSpeakerY) : new Translation2d(FieldConstants.redSpeakerX, FieldConstants.redSpeakerY);
    for (int i = 1; i < DynamicShootingConstants.distanceMapLength - 1; i++) {
      pivotPositions[i] = new Translation2d(DynamicShootingConstants.distanceMap.get(i).get_0(), DynamicShootingConstants.distanceMap.get(i).get_1());
      shooterSpeeds[i] = new Translation2d(DynamicShootingConstants.distanceMap.get(i).get_0(), DynamicShootingConstants.distanceMap.get(i).get_2());      
    }

    ControlVector[] startAndEnd = SplineHelper.getCubicControlVectorsFromWaypoints(
      new Pose2d(
        new Translation2d(
          DynamicShootingConstants.distanceMap.get(0).get_0(),
          DynamicShootingConstants.distanceMap.get(0).get_2()
        ),
        new Rotation2d(DynamicShootingConstants.distanceMap.get(1).get_0() - DynamicShootingConstants.distanceMap.get(0).get_0(), DynamicShootingConstants.distanceMap.get(1).get_2() - DynamicShootingConstants.distanceMap.get(0).get_2())
      ),
      pivotPositions,
      new Pose2d(
        new Translation2d(
          DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 1).get_0(),
          DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 1).get_2()
        ),
        new Rotation2d(DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2).get_0() - DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 1).get_0(), DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2).get_2() - DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 1).get_2())
      )
    );
    
    pivotSplines = SplineHelper.getCubicSplinesFromControlVectors(startAndEnd[0], pivotPositions, startAndEnd[1]);

    startAndEnd = SplineHelper.getCubicControlVectorsFromWaypoints(
      new Pose2d(
        new Translation2d(
          DynamicShootingConstants.distanceMap.get(1).get_0(),
          DynamicShootingConstants.distanceMap.get(1).get_1()
        ),
        new Rotation2d(DynamicShootingConstants.distanceMap.get(1).get_0() - DynamicShootingConstants.distanceMap.get(0).get_0(), DynamicShootingConstants.distanceMap.get(1).get_1() - DynamicShootingConstants.distanceMap.get(0).get_1())
      ),
      pivotPositions,
      new Pose2d(
        new Translation2d(
          DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2).get_0(),
          DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2).get_1()
        ),
        new Rotation2d(DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2).get_0() - DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 1).get_0(), DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2).get_1() - DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 1).get_1())
      )
    );

    shooterSplines = SplineHelper.getCubicSplinesFromControlVectors(startAndEnd[0], pivotPositions, startAndEnd[1]);

  }

  @Override
  public void initialize() {
    swerve.applyRequest(() -> request.withVelocityX(direction * yAxisSupplier.getAsDouble() * 4.5).withVelocityY(direction * xAxisSupplier.getAsDouble() * 4.5).withTargetDirection(swerve.getState().Pose.getTranslation().minus(speakerPose).getAngle()));
  }

  @Override
  public void execute() {

    // Find above and below keys
    double distance = swerve.getState().Pose.getTranslation().minus(speakerPose).getNorm();
    Rotation2d angle = swerve.getState().Pose.getTranslation().minus(speakerPose).getAngle();
    Map.Entry<Double, Integer> lowEntry = DynamicShootingConstants.distanceToIndex.floorEntry(distance);
    Map.Entry<Double, Integer> highEntry = DynamicShootingConstants.distanceToIndex.ceilingEntry(distance);
    
    // Find and apply interpolated angle and speed
    if (lowEntry.getKey() == null || lowEntry.getValue() <= 0 || highEntry.getKey() == null || highEntry.getValue() <= 0) {
      leds.setToHue(1);
      return;
    }
    else {
      leds.setToHue(65);
    }

    double interpolationValue = (distance - lowEntry.getKey())/(highEntry.getKey() - lowEntry.getKey());
    
    // Will get optimised away :)
    double shooterRPM = shooterSplines[lowEntry.getValue()].getPoint(interpolationValue).poseMeters.getY();
    double shooterAngle = pivotSplines[lowEntry.getValue()].getPoint(interpolationValue).poseMeters.getY();

    shooter.setRequest(shooterRPM);
    pivot.setRequest(shooterAngle);

  }

}
