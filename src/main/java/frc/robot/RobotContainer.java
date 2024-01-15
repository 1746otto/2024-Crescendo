// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;



public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final ChassisSpeeds chassisSpeeds = new ChassisSpeeds(5,6, 2);

  private final SwerveRequest.ApplyChassisSpeeds auton = new SwerveRequest.ApplyChassisSpeeds().withSpeeds(chassisSpeeds);
//Choreo stuff
  Field2d m_field = new Field2d();
  ChoreoTrajectory traj;
  ExampleSubsystem subsystem = new ExampleSubsystem();
  Alliance alliance;


  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
    //Choreo stuff 
    traj = Choreo.getTrajectory("NewPath");

    m_field.getObject("traj").setPoses(
      traj.getInitialPose(), traj.getFinalPose()
    );
    m_field.getObject("trajPoses").setPoses(
      traj.getPoses()
    );

  
  }
  public Command getAutonomousCommand() {
    var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

  Command swerveCommand = Choreo.choreoSwerveCommand(
        traj, // Choreo trajectory from above
        () -> (drivetrain.getState().Pose), // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
                                                                                   // translation (input: X error in meters,
                                                                                   // output: m/s).
        new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
                                                                                   // translation (input: Y error in meters,
                                                                                   // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error
        (ChassisSpeeds speeds) -> drivetrain.setControl(auton),//chassisSpeeds.apply(drivetrain.setControl(drive),new SwerveModule[]{new SwerveModule(TunerConstants.FrontLeft, TunerConstants.kCANbusName)}),
        () -> (DriverStation.getAlliance().equals(Alliance.Blue)) ? true : false,
        drivetrain
  
    );


    return Commands.sequence(swerveCommand);
  }

  public void periodic() {
    m_field.setRobotPose(drivetrain.getState().Pose);
  }
}


