// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PrimerConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSubsystemtest;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class RobotContainer {
  // SUBSYSTEMS
  private IntakeSubsystem m_intake = new IntakeSubsystem();
  private IndexerSubsystem m_index = new IndexerSubsystem();
  private ShooterSubsystem m_shooter = new ShooterSubsystem();
  private PrimerSubsystem m_primer = new PrimerSubsystem();

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Vision vision = new Vision(drivetrain);
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

      // Just make sure this doesn't get optimized away.
      vision.toString();
      
      configureBindings();
      
    
  }
 
 

  private void configureBindings() {
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));




    // Intake to primer
    joystick.a().onTrue(new SequentialCommandGroup(
      m_intake.intakeWithCurrentSensingCommand()
      .until(() -> m_intake.isAtReqPosition(IntakeConstants.kOutPosition)),
    new ParallelDeadlineGroup(
      m_primer.PrimeCommand(PrimerConstants.kPrimerPlaceholderSpeed).until(() -> m_primer.isObjectPinchedInPrimer()),
      m_index.indexCommand(),
      m_intake.outtakeCommand().until(() -> !m_intake.isObjectOnHand()).andThen(() -> m_intake.stopIntakingCommand())
    )
    ));
    joystick.a().onFalse(m_index.stopCommand());

    // Basic Intaking with current sensing
    joystick.b().toggleOnTrue(m_intake.intakeWithCurrentSensingCommand());
    joystick.b().toggleOnFalse(m_intake.stopIntakingCommand());


    // Shooting
    joystick.x().onTrue(new ParallelCommandGroup(m_shooter.shootCommand(),
     new SequentialCommandGroup(
      new WaitCommand(2.0),
      m_primer.PrimeCommand(PrimerConstants.kPrimerPlaceholderSpeed)))
     );
    joystick.x().onFalse(new ParallelCommandGroup(m_shooter.stopCommand(), m_primer.StopCommand()));






    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  

  public Command getAutonomousCommand() {
    Command auton1 = drivetrain.getAutoPath("PathPlanTest");
    return auton1;
  }
}