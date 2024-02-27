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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.commands.AmpPosition;
import frc.robot.commands.ShooterPosition;
import frc.robot.commands.podiumPosition;
import frc.robot.commands.subwooferPosition;
import frc.robot.subsystems.LEDSubsystemtest;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  // SUBSYSTEMS

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final LEDSubsystemtest led = new LEDSubsystemtest();
  private final ShooterPivotSubsystem pivot = new ShooterPivotSubsystem();
  private final IntakeRollerSubsystem intakeRollers = new IntakeRollerSubsystem();
  private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem();
  private final PrimerSubsystem primer = new PrimerSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private enum AmpPositionState {Amp, Normal};
  private AmpPositionState ampPosition;
  
 
  public RobotContainer() {
    // Pathplanner event commands
    //Don't initialize any commands before this, it breaks named commands 
    NamedCommands.registerCommand("drivetrainCommand",drivetrain.applyRequest(() -> brake));
    NamedCommands.registerCommand("pivotShooterCommand", pivot.runPivot(Math.PI));
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
    
    // ==========================================================
    // ===================Controller bindings====================
    // ==========================================================

    // Intake rollers stopped and intake stowed
    joystick.a().onTrue(intakeWrist.setStowedPose().alongWith(intakeRollers.stop()));

    // Extend intake, outake rollers once extended, pivot to amp, run shooter, primer, and indexer in reverse
    joystick.b().whileTrue(intakeWrist.setIntakePose().andThen(intakeRollers.outtake())
      .alongWith(pivot.goToAmpPose().andThen(shooter.setToReverse(), primer.setToReverse(), indexer.setToReverse())));
    
    // Toggle climber position
    joystick.x().onTrue(climber.toggle());

    // Extend intake, run rollers in. Intake should automatically stall and stow when note is sensed. Once stowed, intake roller, indexer, and primer run to move note to primed position
    joystick.y().onTrue(intakeWrist.setIntakePose().alongWith(intakeRollers.intakeWithCurrentSensing())
      .andThen(intakeWrist.setStowedPose().alongWith(intakeRollers.hold())).until(() -> intakeWrist.isStowed())
      .andThen(intakeRollers.outtake(), indexer.index(), primer.setToForward()).until(() -> primer.isNoteInShotPosition())
      .andThen(intakeRollers.stop(), indexer.stop(), primer.stop())
      .andThen(primer.setToForward().until(() -> primer.isNoteInShotPosition()))
      .andThen(pivot.goToSetPose()).until(() -> pivot.isAtSetPose())
      .andThen(primer.goToSetPose()));
    
    // Toggles between amp and speaker modes
    joystick.rightBumper().onTrue(pivot.toggleAmpMode().alongWith(primer.toggleAmpMode()));




    
    // Generated Swerve Sim
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