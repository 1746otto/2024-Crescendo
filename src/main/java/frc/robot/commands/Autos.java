// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import java.util.Optional;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;



public final class Autos {
  private double MaxSpeed = 1;
  private double MaxAngularRate = 1.5 * Math.PI;
  SwerveDrivetrain swerve;
  Telemetry telemetry;
  SwerveDriveState state;
  Subsystem subsystem;
  DriverStation driverStation;
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; 
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public Autos(SwerveDrivetrain swerve, SwerveDriveState state) {
   
  
    this.swerve = swerve;
    this.telemetry = telemetry;
    this.state = state;

    
  }
  public Command TestTrajectory(){
   ChoreoTrajectory traj = Choreo.getTrajectory("Test1");
   return Choreo.choreoSwerveCommand(
      traj, // 
      () -> swerve.getState().Pose,
      new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // 
      new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // 
      new PIDController(Constants.AutoConstants.kPThetaController, 0.0, 0.0),  // 
      (ChassisSpeeds speeds) -> {drivetrain.applyRequest(() -> drive.withVelocityX(MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(MaxAngularRate) // Drive counterclockwise with negative X (left)
        );}
          ,
          () -> DriverStation.getAlliance().get() == Alliance.Blue,
          
       subsystem// 
        // 
      );

    

    }
  }



  
