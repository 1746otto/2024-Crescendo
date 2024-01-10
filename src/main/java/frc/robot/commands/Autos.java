// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
  public Command TestTrajectory(){
   ChoreoTrajectory traj = Choreo.getTrajectory("Trajectory");
   return Choreo.choreoSwerveCommand(
      traj, // 
      this::getPose // 
      new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // 
      new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // 
      new PIDController(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // 
      (ChassisSpeeds speeds) -> 
          this.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), ...),
      () -> {
          Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
              mirror = alliance.isPresent() && alliance.get() == Alliance.Red;
      }, // 
      this, // 
  );
    

  }

}

  
