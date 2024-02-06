// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.BooleanSupplier;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private IntakeSubsystem m_intake = new IntakeSubsystem();
  private IndexerSubsystem m_index = new IndexerSubsystem();
  private ShooterSubsystem m_shooter = new ShooterSubsystem();
  private PrimerSubsystem m_primer = new PrimerSubsystem();

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

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // Intake
    joystick.a().onTrue(new SequentialCommandGroup(
      m_intake.IntakeCommand()
      .until(() -> m_intake.isAtReqPosition(IntakeConstants.kOutPosition)),
    new ParallelDeadlineGroup(
      m_primer.PrimeCommand().until(() -> m_primer.objectPinchedInPrimer()).finallyDo(() -> m_primer.StopCommand()),
      m_index.IndexCommand().finallyDo(() -> m_index.StopCommand()),
      m_intake.OuttakeCommand().until(() -> !m_intake.objectOnHand()).finallyDo(() -> m_intake.StopCommand())
    )
    ));

    joystick.b().toggleOnTrue(m_intake.IntakeCommand());
    joystick.b().toggleOnFalse(m_intake.StopCommand());

    // Shooting
    // joystick.b().toggleOnTrue((m_shooter.ShootCommand()));
    // joystick.b().toggleOnFalse(m_shooter.StopCommand());


    joystick.x().onTrue(new ParallelCommandGroup(m_shooter.ShootCommand().finallyDo(() -> m_shooter.StopCommand()),
     new SequentialCommandGroup(
      new WaitCommand(2.0),
      m_primer.PrimeCommand().finallyDo(() -> m_primer.StopCommand())
     )));


    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
