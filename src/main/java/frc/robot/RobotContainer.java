// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.Constants.PrimerConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.ArmRollerSubsystem;
import frc.robot.subsystems.LEDSubsystemtest;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class RobotContainer {
  // SUBSYSTEMS

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private IntakeRollerSubsystem m_intakeRollers = new IntakeRollerSubsystem();
  private IntakeWristSubsystem m_intakeWristSubsystem = new IntakeWristSubsystem();
  private IndexerSubsystem m_index = new IndexerSubsystem();
  private ShooterSubsystem m_shooter = new ShooterSubsystem();
  private PrimerSubsystem m_primer = new PrimerSubsystem();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final LEDSubsystemtest led = new LEDSubsystemtest();
  private final ShooterPivotSubsystem pivot = new ShooterPivotSubsystem();
  

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  
 
  //pathplanner testing
  public RobotContainer() {
    //Don't initialize any commands before this, it breaks named commands 
      NamedCommands.registerCommand("drivetrainCommand",drivetrain.applyRequest(() -> brake));
      NamedCommands.registerCommand("pivotShooterCommand", pivot.runPivot(Math.PI));

      NamedCommands.registerCommand("intakeCommand", new ParallelCommandGroup(m_intakeWristSubsystem.runWrist(),m_intakeRollers.runRollerCommand()).withTimeout(2));
      //Change timeout
      //NamedCommands.registerCommand("shootCommand", m_intake.outtakeCommand().withTimeout(2.5));
      //NamedCommands.registerCommand("drivetrainCommand",drivetrain.applyRequest(() -> brake));
      configureBindings();
      
    
  }
 
 

  private void configureBindings() {
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    //Testing intake, primer, and shooter
    joystick.a().whileTrue(new ParallelCommandGroup(m_intakeWristSubsystem.runWrist(),m_intakeRollers.runRollerCommand()));
    joystick.a().whileFalse(m_intakeRollers.runStopCommand());
    joystick.b().whileTrue(m_primer.ForwardCommand());
    joystick.b().whileFalse(m_primer.StopCommand());
    joystick.x().whileTrue(m_shooter.shootCommand());
    joystick.x().whileFalse(m_shooter.stopCommand());




    //joystick.a().onFalse(m_index.stopCommand());

    // Basic Intaking/Shooting to test
    //joystick.a().whileTrue(new RunCommand(() -> m_intakeWristSubsystem.testIntake(), m_intakeWristSubsystem));
    //joystick.a().whileTrue(new RunCommand(() -> m_shooter.runShooterRollers(0.1), m_shooter));





    // Shooting
    // joystick.x().onTrue(new ParallelCommandGroup(m_shooter.shootCommand(),
    //  new SequentialCommandGroup(
    //   new WaitCommand(2.0),
    //   m_primer.PrimeCommand(PrimerConstants.kPrimerPlaceholderSpeed)))
    //  );
    // joystick.x().onFalse(new ParallelCommandGroup(m_shooter.stopCommand(), m_primer.StopCommand()));






    joystick.x().whileTrue(new RunCommand(() -> pivot.testShooter(), pivot));
    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  

  public Command getAutonomousCommand() {
    // Command auton = drivetrain.getAutoPath("5Piece");
    // Command baseAuton1 = drivetrain.getAutoPath("Base Auton1");
    // Command baseAuton2 = drivetrain.getAutoPath("Base Auton2");
    // Command baseAuton3 = drivetrain.getAutoPath("Base Auton3");
    // Command theory = drivetrain.getAutoPath("ThreeSouthSide");
    //return theory;
    return Commands.print("Placeholder");
  }
}