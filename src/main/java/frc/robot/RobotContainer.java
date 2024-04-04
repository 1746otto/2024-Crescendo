// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.BackpackRollerSubsystem;
import frc.robot.subsystems.BackpackWristSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.Constants.BackpackRollerConstants;
import frc.robot.Constants.BackpackWristConstants;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.ShooterWristConstants;
import frc.robot.Constants.TeleopSwerveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShootAnywhereAuton;
import frc.robot.commands.ShootAnywhereCommand;
import frc.robot.commands.ShootStaticAuton;
import frc.robot.commands.handleLEDCommand;
import frc.robot.commands.handlePrimerShooter;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class RobotContainer {
  // SUBSYSTEMS

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public double power = TeleopSwerveConstants.SwerveMagnitudeExponent;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  //public final Vision vision = new Vision(drivetrain);
  private final LEDSubsystem led = new LEDSubsystem();
  private final ShooterPivotSubsystem pivot = new ShooterPivotSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeRollerSubsystem intakeRollers = new IntakeRollerSubsystem();
  private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem();
  private final PrimerSubsystem primer = new PrimerSubsystem();
  private final BackpackWristSubsystem backpackWrist = new BackpackWristSubsystem();
  private final BackpackRollerSubsystem backpackRoller = new BackpackRollerSubsystem();

  private final SwerveRequest.FieldCentric drive = TeleopSwerveConstants.TeleopDriveRequest;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(TeleopSwerveConstants.MaxSpeedMetersPerSec);

  private enum AmpPositionState {
    Amp, Normal
  };

  public boolean backPackToggle;

  private AmpPositionState ampPosition = AmpPositionState.Normal;
  public SendableChooser<String> autoChooser;
  public Command autonCommand;
  public String currentAuton;

  public BooleanSupplier inIntakeUp = (() -> intakeRollers.intakeHasPiece()
      && intakeWrist.isAtReqPosition(IntakeWristConstants.kStow));
  public BooleanSupplier inIntakeDown = (() -> intakeRollers.intakeHasPiece()
      && intakeWrist.isAtReqPosition(IntakeWristConstants.kIntake));
  public BooleanSupplier notInIntakeDown = (() -> !intakeRollers.intakeHasPiece()
      && intakeWrist.isAtReqPosition(IntakeWristConstants.kStow));
  public BooleanSupplier notInIntake = (() -> !intakeRollers.intakeHasPiece()
      && intakeWrist.isAtReqPosition(IntakeWristConstants.kStow));
  public BooleanSupplier inShooter = (() -> primer.isPrimerBeamBreakBroken());
  public boolean isIndexing = false;
  public double temp = -1;
  boolean intookPiece;
  boolean stowPivot = false;

  // pathplanner testing
  public RobotContainer() {
    NamedCommands.registerCommand("slamIntakeCommand", intakeWrist.intakePosCommand());
    /* 
     * Needs 2.375 seconds. 
     * Intaking period is .75 seconds.
     * Down approximately a half second after its called.
     * Intaking takes .5 seconds from piece touch.
     * 1 Second for it to return to flat after intaking is complete.
    */
    NamedCommands.registerCommand("intakeCommand", 
      new SequentialCommandGroup(
        intakeRollers.intakeSpeedCommand(),
        intakeWrist.intakePosCommand()
          .withTimeout(.5),
        new WaitUntilCommand(() -> intakeRollers.intakeHasPiece())
          .withTimeout(.75),
        intakeRollers.stowSpeedCommand(),
        new ParallelCommandGroup(
          intakeWrist.indexPosCommand(),
          pivot.goToIntakePos()
        ).withTimeout(.6),
        new ParallelCommandGroup(
          primer.intakeCommand(),
          intakeRollers.outtakeCommand()
        ).withTimeout(.325), // Stops primer by itself
        intakeRollers.stopCommand()
      )
        .finallyDo(
          () -> {
            intakeRollers.stop();
            pivot.setRequest(ShooterWristConstants.kFlat);
          }
        )
      );
    
    NamedCommands.registerCommand("prepToShootMidStage", 
      new ParallelCommandGroup(
        pivot.runPivot(ShooterWristConstants.kMidStagePos),
        shooter.setRequestCommand(ShooterConstants.kMidStageSpeed)
      )
    );

    NamedCommands.registerCommand("finishIndex",
      new ParallelCommandGroup(
        intakeRollers.outtakeCommand(),
        primer.intakeCommand(),
        pivot.goToIntakePos()
      )
        .withTimeout(.15)
        .onlyIf(() -> !primer.isPrimerBeamBreakBroken())
    );

    NamedCommands.registerCommand("shootPieceMidStage",
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          pivot.runPivot(ShooterWristConstants.kMidStagePos),
          new WaitUntilCommand(shooter::isAtReq)
            .withTimeout(.5)
        ),
        primer.setSpeedCommand(1),
        new WaitUntilCommand(primer::isNoteShot)
          .withTimeout(.3),
        primer.stopCommand()
      )
        .finallyDo(() -> pivot.setRequest(ShooterWristConstants.kFlat))
    );


    //Named Commands for 4P Fixed
    NamedCommands.registerCommand("pivotSubwoofer", pivot.goToSubCommand().withTimeout(.5));
    NamedCommands.registerCommand("pivotToIntake", pivot.goToIntakePos().withTimeout(.25));
    NamedCommands.registerCommand("goToSubwooferSpeed", shooter.setRequestCommand(4000).withTimeout(1));
    NamedCommands.registerCommand("shootPiece", primer.setSpeedCommand(PrimerConstants.kShoot).until(() -> primer.isNoteShot()).withTimeout(.5).finallyDo(() -> primer.stop()));

    //Shoot ANYWHERE
    NamedCommands.registerCommand("shootPieceAnywhere", new ShootAnywhereAuton(drivetrain, shooter, pivot, led, primer).until(() -> primer.isNoteShot()));
    
    //Shooting commands should work
    NamedCommands.registerCommand("shootPiece1", new ShootStaticAuton(pivot, primer, -0.345)); //NEED VALUE HERE
    NamedCommands.registerCommand("primeShooterRoller1", shooter.setRequestCommand(5200));//NEED VALUE HERE

    NamedCommands.registerCommand("shootPiece2", new ShootStaticAuton(pivot, primer, -0.345)); //NEED VALUE HERE
    NamedCommands.registerCommand("primeShooterRoller2", shooter.setRequestCommand(5200));//NEED VALUE HERE

    NamedCommands.registerCommand("shootPiece3", new ShootStaticAuton(pivot, primer, -0.345)); //NEED VALUE HERE
    NamedCommands.registerCommand("primeShooterRoller3", shooter.setRequestCommand(5200));//NEED VALUE HERE
    //static positions


    configureBindings();
    configureDefaultCommands();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Middle subwoofer two piece", "Middle2P");
    // autoChooser.addOption("Top subwoofer (Amp side) two piece", "Top2P");
    autoChooser.addOption("Middle subwoofer two piece", "Middle2P");
    // autoChooser.addOption("Bottom subwoofer (Source side) two piece", "Bottom2P");
    autoChooser.addOption("Four Piece close. Start center subwoofer", "4 Piece Fixed");
    
    // autoChooser.addOption("Two piece south. Start on source side", "2PSouth");
    // autoChooser.addOption("Four piece south. Start on source side", "4PSouthPreload");

  }

  private void configureBindings() {


    
     drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
     drivetrain.applyRequest(() -> drive.withVelocityX(temp * joystick.getLeftY() *
     Math.pow(Math.abs(joystick.getLeftY()), TeleopSwerveConstants.SwerveMagnitudeExponent - 1) * TeleopSwerveConstants.MaxSpeedMetersPerSec) // Drive
     //forward with
     // negative Y (forward)
     .withVelocityY(temp * joystick.getLeftX() *
     Math.pow(Math.abs(joystick.getLeftX()), TeleopSwerveConstants.SwerveMagnitudeExponent - 1) * TeleopSwerveConstants.MaxSpeedMetersPerSec) // Drive left
     //with negative X (left)
     .withRotationalRate(-joystick.getRightX() *
     Math.pow(Math.abs(joystick.getRightX()), TeleopSwerveConstants.SwerveMagnitudeExponent - 1) * TeleopSwerveConstants.MaxAngularRateRotPerSec) //
    // Drive counterclockwise with negative X (left)
     ));
     
    // dumb commands to test

    // joystick.y().onTrue(pivot.goToIntakePos());
    // joystick.a().whileTrue(primer.intakeCommand());
    // joystick.a().whileFalse(primer.stopCommand());
    // joystick.b().whileTrue(shooter.ShootCommand());
    // joystick.b().whileFalse(shooter.StopCommand());
    // joystick.x().onTrue(new ParallelDeadlineGroup(intakeWrist.intakePosCommand(),
    // intakeRollers.intakeSpeedCommand()));

    
    joystick.y().onTrue(
    new SequentialCommandGroup(
    new InstantCommand(() -> {primer.primerStow = false;}), // Currently  unnecessary may be used if we need to fix stow idk man
    new ParallelDeadlineGroup(
    intakeWrist.intakePosCommand(),
    intakeRollers.intakeSpeedCommand(),
    pivot.goToIntakePos(),
    new InstantCommand(() -> ampPosition = AmpPositionState.Normal)
    ),
    new WaitUntilCommand(() -> intakeRollers.intakeHasPiece())
    .withTimeout(5),
    intakeRollers.stopCommand(),
    
    new ParallelDeadlineGroup(
    primer.intakeCommand(),
    intakeWrist.indexPosCommand().andThen(intakeRollers.outtakeCommand()) // Stops primer by itself
    )
   
    )
     .finallyDo(
    () -> {
    intakeRollers.stop();
    pivot.setRequest(ShooterWristConstants.kFlat);
        }
      )
    );

    // Temporary test button for autonomous
    // joystick.leftTrigger().and(() -> primer.isPrimerBeamBreakBroken()).whileTrue(
    //     shootAnywhereCommand);

    // joystick.y().and(notInIntakeDown).onTrue( //Change for toggling
    // new SequentialCommandGroup(
    // intakeWrist.indexPosCommand(),
    // new ParallelDeadlineGroup(
    // primer.intakeCommand(), // Stops primer by itself
    // intakeRollers.outtakeCommand()
    // )).finallyDo(
    // () -> {
    // intakeRollers.stop();
    // pivot.setRequest(ShooterWristConstants.kFlat);
    // }
    // ));

    // joystick.a().whileTrue(primer.setIntakeSpeed().finallyDo(() ->
    // primer.setSpeed(0)));

    // joystick.a().onTrue(new InstantCommand(() ->
    // {primer.setSpeed(.5);}).andThen(new WaitCommand(.2)).andThen(new
    // InstantCommand(() ->
    // {primer.setSpeed(0);})).andThen(primer.backupCommand()));

    joystick.x().onTrue(
        new ParallelCommandGroup(
            new InstantCommand(
                () -> {
                  intakeRollers.stop();
                  primer.stop();
                }),
            intakeWrist.indexPosCommand()));

    joystick.start().onTrue(
        new InstantCommand(
            () -> drivetrain.seedFieldRelative(
                new Pose2d(
                    drivetrain.getState().Pose.getTranslation(),
                    Rotation2d.fromDegrees(/* (temp == 1) ? 180 : */0)))));

    joystick.povUp().toggleOnTrue(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          pivot.goToBackpackPos(),
          backpackWrist.goToBackPackPos(),
          shooter.setRequestCommand(ShooterConstants.kBackpackSpeed)
        ).withTimeout(1),
        new StartEndCommand(() -> {
          backpackRoller.setRollerSpeed(BackpackRollerConstants.kIntake);
          primer.setSpeed(PrimerConstants.kBackpackSpeed);
        }, () -> {
          shooter.stop();
          backpackRoller.stop();
          primer.stop();
          pivot.setRequest(ShooterWristConstants.kFlat);
          backpackWrist.setRequest(BackpackWristConstants.kStow);
        })));
    
    // joystick.b().onTrue(
    //   new SequentialCommandGroup(
    //     pivot.goToAmpPosition
    //     intakeWrist.ampPosCommand(),
    //     intakeRollers.ampCommand().andThen(new WaitUntilCommand(() -> joystick.getHID().getBButtonReleased()))
    //   )
    //   .andThen(intakeWrist.indexPosCommand().alongWith(intakeRollers.stopCommand()))
    // );
    // joystick.b().whileTrue(
    //   new SequentialCommandGroup(
    //     pivot.goToAmpPosition(),
    //     intakeWrist.ampPosCommand(),
    //     intakeRollers.ampCommand())
    //   );
    // joystick.b().onFalse(
    //   intakeWrist.indexPosCommand().andThen(intakeRollers.stopCommand())
    // );
  
     
    // joystick.a().onTrue( //Test button
    // new SequentialCommandGroup(
    // new InstantCommand(() -> {primer.primerStow = false;}), // Currently  unnecessary may be used if we need to fix stow idk man
    // new ParallelDeadlineGroup(
    // intakeWrist.intakePosCommand(),
    // intakeRollers.intakeSpeedCommand(),
    // pivot.goToIntakePos(),
    // new InstantCommand(() -> ampPosition = AmpPositionState.Normal)
    // ),
    // new WaitUntilCommand(() -> intakeRollers.intakeHasPiece())
    // .withTimeout(5),
    // intakeRollers.stopCommand(),
    // intakeWrist.indexPosCommand(),
    // shooter.ShootCommand(),
    // new ParallelDeadlineGroup(
    // primer.intakeCommand(), // Stops primer by itself
    // intakeRollers.outtakeCommand()),
    // intakeRollers.stopCommand()
    // )
    //  .finallyDo(
    // () -> {
    // intakeRollers.stop();
    // pivot.setRequest(ShooterWristConstants.kFlat);
    //     }
    //   )
    // );
    // pivot
    // joystick.rightBumper().and(() -> primer.isPrimerBeamBreakBroken() ||
    // joystick.getHID().getAButtonPressed()).toggleOnTrue(pivot.goToAmpPose()/*.andThen(new
    // WaitCommand(100))*/.alongWith(new InstantCommand(() -> ampPosition =
    // AmpPositionState.Amp))/*.finallyDo(() ->
    // pivot.setRequest(ShooterWristConstants.kStartPos))*/);
    // joystick.leftTrigger().and(() -> primer.isPrimerBeamBreakBroken() ||
    // joystick.getHID().getAButtonPressed()).whileTrue(pivot.goToPodiumPos().alongWith(new
    // InstantCommand(() ->
    // intakeWrist.setRequest(IntakeWristConstants.kIntake))).alongWith(shooter.setRequestCommand(ShooterConstants.kShoot).alongWith(new
    // WaitUntilCommand(100))).finallyDo(() -> {shooter.stop();
    // pivot.goToIntakePos();
    // intakeWrist.setRequest(IntakeWristConstants.kStow);}));
    joystick.leftTrigger().and(() -> primer.isPrimerBeamBreakBroken() || joystick.getHID().getAButtonPressed())
        .whileTrue(pivot.goToSubCommand().alongWith(shooter.setRequestCommand(ShooterConstants.kSubwooferSpeed))
            .alongWith(new InstantCommand(() -> ampPosition = AmpPositionState.Normal)).alongWith(new WaitCommand(100))
            .finallyDo(() -> {
              shooter.stop();
              pivot.setRequest(ShooterWristConstants.kFlat);
            }));
    joystick.leftBumper().and(() -> primer.isPrimerBeamBreakBroken() || joystick.getHID().getAButtonPressed())
        .whileTrue(pivot.goToFerryPos().alongWith(shooter.setRequestCommand(ShooterConstants.kFerry))
            .alongWith(new InstantCommand(() -> ampPosition = AmpPositionState.Normal)).alongWith(new WaitCommand(100))
            .finallyDo(() -> {
              shooter.stop();
              pivot.setRequest(ShooterWristConstants.kFlat);
            }));
    joystick.rightTrigger().whileTrue(primer.setSpeedCommand(PrimerConstants.kShoot).andThen(new RunCommand(() -> {
    })).finallyDo(() -> {
      primer.primerStow = primer.isPrimerBeamBreakBroken();
    }));
    joystick.rightTrigger().onFalse(primer.backupCommand());

    // joystick.povLeft().whileTrue(new StartEndCommand(() -> pivot.test(), () -> pivot.stop(), pivot));
    // joystick.povRight().onTrue(pivot.goToIntakePos());

    if (Utils.isSimulation()) {

      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void configureDefaultCommands() {
    led.setDefaultCommand(new handleLEDCommand(led, () -> intakeRollers.intakeHasPiece(), inShooter).ignoringDisable(true));
    // pivot.setDefaultCommand(pivot.goToParallelPos().onlyIf(notInIntakeDown));
    // check wrist up and intake roller beambreak is triggered
  }

  public void setTeleopInitState() {
    shooter.stop();
    intakeRollers.stop();
    intakeWrist.setRequest(IntakeWristConstants.kStow);
    primer.stop();
    pivot.setRequest(ShooterWristConstants.kFlat);
    // vision.stopThread();
  }

  public Command getAutonomousCommand() {
  // Command auton = drivetrain.getAutoPath("5Piece");
  // Command baseAuton1 = drivetrain.getAutoPath("Base Auton1");
  // Command middle2P = drivetrain.getAutoPath("Middle2P");
  // Command bottom2P = drivetrain.getAutoPath("Bottom2P");
  // Command baseAuton3 = drivetrain.getAutoPath("Base Auton3");
  // Command theory = drivetrain.getAutoPath("Bottom4P");
  // Command top2Piece = drivetrain.getAutoPath("Top2P");
  // return theory;
  //Command test = drivetrain.getAutoPath("testAuto2");
  return new InstantCommand();
  }
}