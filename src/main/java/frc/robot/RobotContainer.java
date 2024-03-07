// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.ShooterWristConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AmpPosition;
import frc.robot.commands.ShooterPosition;
import frc.robot.commands.TeleopIntakeToPrimerCommand;
import frc.robot.commands.checkPrimerPiece;
import frc.robot.commands.handlePrimerShooter;
import frc.robot.commands.podiumPosition;
import frc.robot.commands.subwooferPosition;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class RobotContainer {
  // SUBSYSTEMS

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Vision vision = new Vision(drivetrain);
  private final LEDSubsystem led = new LEDSubsystem();
  private final ShooterPivotSubsystem pivot = new ShooterPivotSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeRollerSubsystem intakeRollers = new IntakeRollerSubsystem();
  private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem();
  private final PrimerSubsystem primer = new PrimerSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private enum AmpPositionState {Amp, Normal};
  private AmpPositionState ampPosition = AmpPositionState.Normal;
  
 
  //pathplanner testing
  public RobotContainer() {

    //Don't initialize any commands before this, it breaks named commands
    
    NamedCommands.registerCommand("intakeCommand", new ParallelDeadlineGroup(intakeRollers.intakeCommand(), intakeWrist.intakePosCommand())
    .andThen(new ParallelDeadlineGroup(primer.intakeCommand(), //Deadline
    new SequentialCommandGroup(intakeWrist.indexPosCommand().alongWith(indexer.forwardCommand()),intakeRollers.outtakeCommand()))).withTimeout(2));
    NamedCommands.registerCommand("pivotPodium", pivot.runPivot(ShooterWristConstants.kPodiumPos));
    NamedCommands.registerCommand("pivotAmp", pivot.runPivot(ShooterWristConstants.kAmpPos));
    NamedCommands.registerCommand("pivotIntakePos", pivot.runPivot(ShooterWristConstants.kIntakePos));
    NamedCommands.registerCommand("pivotSubwoofer", pivot.runPivot(ShooterWristConstants.kSubwooferPos));
    NamedCommands.registerCommand("primeShooter", new handlePrimerShooter(primer, () -> ampPosition == AmpPositionState.Amp));
    
    // Just make sure this doesn't get optimized away.
    vision.toString();

    configureBindings();
    configureDefaultCommands();
      
    
  }
 
 

  private void configureBindings() {
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
         ));

    joystick.y().toggleOnTrue(intakeWrist.intakePosCommand().withTimeout(2).andThen(intakeWrist.indexPosCommand()));

    joystick.rightBumper().toggleOnTrue(pivot.goToAmpPose().alongWith(new InstantCommand(() -> ampPosition = AmpPositionState.Amp)));
    joystick.leftTrigger().whileTrue(pivot.goToPodiumPos());
    joystick.leftTrigger().whileFalse(pivot.goToNormalPos());
    joystick.rightTrigger().whileTrue(shooter.ShootCommand().alongWith(new WaitCommand(1).andThen(new handlePrimerShooter(primer,() -> ampPosition == AmpPositionState.Amp))));

    joystick.povCenter().onTrue(new ParallelDeadlineGroup(AutoBuilder.pathfindToPose((DriverStation.getAlliance().get() == Alliance.Blue) ? FieldConstants.blueAmpPose : FieldConstants.redAmpPose, AutoConstants.dynamicPlanningPathConstraints), pivot.goToAmpPose().alongWith(new InstantCommand(() -> ampPosition = AmpPositionState.Amp))));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }
  public void configureDefaultCommands() {
    indexer.setDefaultCommand(indexer.forwardCommand().onlyIf(() -> (intakeRollers.isIntakeBeamBreakBroken() && intakeWrist.isAtReqPosition(IntakeWristConstants.kStow) && !primer.isPrimerBeamBreakBroken())));
    intakeRollers.setDefaultCommand(intakeRollers.outtakeCommand().onlyIf(() -> (!intakeRollers.isIntakeBeamBreakBroken() && intakeWrist.isAtReqPosition(IntakeWristConstants.kStow) && !primer.isPrimerBeamBreakBroken())));
    primer.setDefaultCommand(primer.intakeCommand().onlyIf(() -> (intakeRollers.isIntakeBeamBreakBroken() && intakeWrist.isAtReqPosition(IntakeWristConstants.kStow) && !primer.isPrimerBeamBreakBroken()))); // check wrist up and intake roller beambreak is triggered
  }

  

  public Command getAutonomousCommand() {
    // Command auton = drivetrain.getAutoPath("5Piece");
    // Command baseAuton1 = drivetrain.getAutoPath("Base Auton1");
    // Command baseAuton2 = drivetrain.getAutoPath("Base Auton2");
    // Command baseAuton3 = drivetrain.getAutoPath("Base Auton3");
    Command theory = drivetrain.getAutoPath("ThreeSouthSide");
    //return theory;
    Command tune = drivetrain.getAutoPath("PathPlanTest");
    Command baseAuton4 = drivetrain.getAutoPath("4Piece");
    return theory;
  }
}