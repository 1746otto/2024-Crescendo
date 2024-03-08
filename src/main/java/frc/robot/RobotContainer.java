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
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.ShooterWristConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TeleopIntakeToPrimerCommand;
import frc.robot.commands.checkPrimerPiece;
import frc.robot.commands.handlePrimerShooter;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.BooleanSupplier;

import javax.swing.GroupLayout.ParallelGroup;

import frc.robot.subsystems.ArmRollerSubsystem;
import frc.robot.subsystems.LEDSubsystemtest;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class RobotContainer {
  // SUBSYSTEMS

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final LEDSubsystemtest led = new LEDSubsystemtest();
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

  public BooleanSupplier inIntakeUp = (() -> intakeRollers.isBeamBreakTriggered() && intakeWrist.isAtReqPosition(IntakeWristConstants.kStow));
  public BooleanSupplier inIntakeDown = (() -> intakeRollers.isBeamBreakTriggered() && intakeWrist.isAtReqPosition(IntakeRollerConstants.kHold));
  public BooleanSupplier inShooter = (() -> primer.isPrimerBeamBreakBroken());
  
 
  //pathplanner testing
  public RobotContainer() {
      NamedCommands.registerCommand("intakeCommand", new ParallelDeadlineGroup(intakeRollers.intakeCommand(), intakeWrist.intakePosCommand()).withTimeout(1.5)
      .andThen(new ParallelDeadlineGroup(primer.intakeCommand(), //Deadline
      intakeWrist.indexPosCommand().alongWith(indexer.forwardCommand(),intakeRollers.outtakeCommand())))); //Change to andThen if broken
      NamedCommands.registerCommand("pivotPodium", pivot.runPivot(ShooterWristConstants.kPodiumPos));
      NamedCommands.registerCommand("pivotAmp", pivot.runPivot(ShooterWristConstants.kAmpPos));
      NamedCommands.registerCommand("pivotIntakePos", pivot.runPivot(ShooterWristConstants.kIntakePos));
      NamedCommands.registerCommand("pivotSubwoofer", pivot.runPivot(ShooterWristConstants.kSubwooferPos));
      NamedCommands.registerCommand("primeShooter", new handlePrimerShooter(primer, () -> ampPosition == AmpPositionState.Amp).withTimeout(1));
      NamedCommands.registerCommand("ShootSubWoofer", shooter.setSpeedCommand(ShooterConstants.kSubwooferShot));
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

    joystick.y().onTrue(new ParallelCommandGroup(new RepeatCommand(new InstantCommand()), new InstantCommand(() -> intakeRollers.setSpeed(IntakeRollerConstants.kIntake)), intakeWrist.intakePosCommand(), pivot.goToNormalPos().alongWith(new InstantCommand(() -> ampPosition = AmpPositionState.Normal)))
      .until(() -> intakeWrist.isAtReqPosition(IntakeWristConstants.kIntake) && intakeRollers.intakeHasPiece())
      .andThen(new ParallelDeadlineGroup(intakeWrist.indexPosCommand(), indexer.setSpeedCommand(IndexerConstants.kForward)))
      .andThen(primer.intakeCommand().deadlineWith(intakeRollers.outtakeCommand())).andThen(intakeRollers.stopCommand().alongWith(indexer.stopCommand())));
    //Fixed controls
    

    //pivot
    joystick.rightBumper().and(inShooter).toggleOnTrue(pivot.goToAmpPose().alongWith(new InstantCommand(() -> ampPosition = AmpPositionState.Amp)));
    joystick.leftTrigger().and(inShooter).whileTrue(pivot.goToPodiumPos().alongWith(shooter.setSpeedCommand(ShooterConstants.kShoot)));
    joystick.leftTrigger().and(inShooter).whileFalse(pivot.goToNormalPos().alongWith(shooter.StopCommand()));
    joystick.leftBumper().and(inShooter).whileTrue(pivot.goToSubCommand().alongWith(shooter.setSpeedCommand(ShooterConstants.kSubwooferShot)).alongWith(new InstantCommand(() -> ampPosition = AmpPositionState.Normal)));
    joystick.leftBumper().and(inShooter).whileFalse(pivot.goToNormalPos().alongWith(shooter.StopCommand()));
    joystick.rightTrigger().whileTrue(new handlePrimerShooter(primer,() -> ampPosition == AmpPositionState.Amp));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }
  public void configureDefaultCommands() {
     // check wrist up and intake roller beambreak is triggered
  }

  

  public Command getAutonomousCommand() {
    // Command auton = drivetrain.getAutoPath("5Piece");
    Command baseAuton1 = drivetrain.getAutoPath("Base Auton1");
    // Command baseAuton2 = drivetrain.getAutoPath("Base Auton2");
    // Command baseAuton3 = drivetrain.getAutoPath("Base Auton3");
    Command theory = drivetrain.getAutoPath("ThreeSouthSide");
    //return theory;
    Command tune = drivetrain.getAutoPath("PathPlanTest");
    Command baseAuton4 = drivetrain.getAutoPath("4Piece");
    return baseAuton1;
  }
}