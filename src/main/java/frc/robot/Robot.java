// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.EmptyStackException;
import java.util.Optional;
import java.util.Random;
import java.util.function.Supplier;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionSimConstants;
import frc.robot.subsystems.VisionSim;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Optional<VisionSim> visionSimSubsystem;
  private Optional<SwerveDrivePoseEstimator> poseEstimator;
  private VisionSystemSim visionSim = new VisionSystemSim("photonSim2");
  private Pose3d simPose = new Pose3d(new Pose2d(new Translation2d(2.446, 5.854), new Rotation2d(0)));
  private Pose3d badStartPose = new Pose3d();
  private TargetModel model36h11 = TargetModel.kAprilTag36h11;
  private VisionTargetSim visionTargetSim = new VisionTargetSim(new Pose3d(), model36h11);
  private AprilTagFieldLayout simAprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  private Supplier<Rotation3d> gyro = () -> simPose.getRotation();
  private Random encoderNoise = new Random();
  private ArrayList<Double> noiseOverTime = new ArrayList<Double>();
  PathPlannerTrajectory autoSimTraj;
  boolean isAuton = false;
  double autoStart;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData(m_robotContainer.autoChooser);
    if (!Utils.isSimulation()) {
      m_robotContainer.vision.startThread();
      return;
    }

    visionSim.addAprilTags(simAprilTagFieldLayout);
    visionSim.addVisionTargets(visionTargetSim);
    visionSim.resetRobotPose(simPose);

    poseEstimator = Optional.of(new SwerveDrivePoseEstimator(VisionSimConstants.Kinematics.getValue(),
        simPose.toPose2d().getRotation(), VisionSimConstants.SwerveModulePositions.getValue(),
        badStartPose.toPose2d()));
    visionSimSubsystem = Optional
        .of(new VisionSim(poseEstimator.get(), gyro, visionSim, VisionSimConstants.CameraProperties.getValue()));
    poseEstimator.get().resetPosition(gyro.get().toRotation2d(), VisionSimConstants.SwerveModulePositions.getValue(),
        badStartPose.toPose2d());

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (m_robotContainer.autoChooser.getSelected() != m_robotContainer.currentAuton) {
      m_robotContainer.autonCommand = m_robotContainer.drivetrain
          .getAutoPath(m_robotContainer.autoChooser.getSelected());
      m_robotContainer.currentAuton = m_robotContainer.autoChooser.getSelected();
    }
  }

  @Override
  public void disabledExit() {
  }

  private void setVisionConstants() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        VisionConstants.kSpeakerId = 7;
        VisionConstants.kSpeakerPose = new Translation2d(FieldConstants.blueSpeakerX, FieldConstants.blueSpeakerY);
      } else {
        VisionConstants.kSpeakerId = 3;
        VisionConstants.kSpeakerPose = new Translation2d(FieldConstants.redSpeakerX, FieldConstants.redSpeakerY);
      }
    } else {
      VisionConstants.kSpeakerId = 7;
      VisionConstants.kSpeakerPose = new Translation2d(FieldConstants.blueSpeakerX, FieldConstants.blueSpeakerY);
    }
  }

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    setVisionConstants();

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    setVisionConstants();
    m_robotContainer.setTeleopInitState();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationInit() {
    PathPlannerPath thing = PathPlannerPath.fromChoreoTrajectory("4PUnderStage");
    autoSimTraj = thing.getTrajectory(new ChassisSpeeds(), thing.getPreviewStartingHolonomicPose().getRotation());

  }

  private double lastTimestamp = Timer.getFPGATimestamp();

  @Override
  public void simulationPeriodic() {
    boolean simLocalization = false;
    setVisionConstants();
    if (simLocalization) {
      localizationSimulation();
    } else {
      shootAnywhereSimulation();
    }
  }

  private void shootAnywhereSimulation() {
    visionSim.update(simPose);
    if (!visionSimSubsystem.get().isRunning()) {
      visionSimSubsystem.get().startThread();
    }
    if (Timer.getFPGATimestamp() - visionSimSubsystem.get().lastResults[0].getTimestampSeconds() > .3
        || !visionSimSubsystem.get().containsSpeakerTag(0)) {
      return;
    } else {
      Rotation2d calculatedAngle = VisionConstants.kSpeakerPose.minus(visionSimSubsystem.get().cameraPoses[0].getTranslation().toTranslation2d())
              .getAngle().plus(Rotation2d.fromDegrees(180));
              Rotation2d actualAngle = VisionConstants.kSpeakerPose.minus(simPose.getTranslation().toTranslation2d())
              .getAngle().plus(Rotation2d.fromDegrees(180));
      System.out.println("Calculated angle: " + 
          calculatedAngle);
      System.out.println("Actual angle: " + actualAngle);
      System.out.println("Error (deg): " + calculatedAngle.minus(actualAngle).getDegrees());
    }
  }

  private void localizationSimulation() {
    Transform2d diff = simPose.toPose2d().minus(poseEstimator.get().getEstimatedPosition());
    noiseOverTime.add(Math.pow(diff.getX(), 2) + Math.pow(diff.getY(), 2));
    if (!visionSimSubsystem.get().isRunning()) {
      visionSimSubsystem.get().startThread();
    }

    if (DriverStation.isAutonomous() && !isAuton) {
      isAuton = true;
      autoStart = Timer.getFPGATimestamp();
      simPose = new Pose3d(autoSimTraj.sample(0).getTargetHolonomicPose());
      poseEstimator.get().resetPosition(gyro.get().toRotation2d(),
          VisionSimConstants.SwerveModulePositions.getValue(),
          simPose.toPose2d());
    }
    if (!DriverStation.isAutonomous() && isAuton) {
      isAuton = false;
    }
    var lastPose = simPose;
    var overallAutonDelta = Timer.getFPGATimestamp() - autoStart;
    var timeDelta = Timer.getFPGATimestamp() - lastTimestamp;
    var sample = autoSimTraj.sample(overallAutonDelta);
    if (DriverStation.isAutonomous()) {
      simPose = new Pose3d(sample.getTargetHolonomicPose());
    } else {
      Transform3d oneDegreeTransform = new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.toRadians(1)));
      simPose = simPose.transformBy(oneDegreeTransform);
    }
    var difference = lastPose.minus(simPose);
    var calculated = difference.getRotation().getAngle() / timeDelta;
    System.out.println("given: " + sample.holonomicAngularVelocityRps.get());
    System.out.println("calculated : " + calculated);
    System.out.println("given x: " + sample.velocityMps * sample.heading.getCos());
    System.out.println("calculated x: " + difference.getX() / timeDelta);
    System.out.println("given y: " + sample.velocityMps * sample.heading.getSin());
    System.out.println("calculated y: " + difference.getY() / timeDelta);
    ChassisSpeeds speeds = new ChassisSpeeds(-difference.getX() / timeDelta, -difference.getY() / timeDelta,
        calculated);
    System.out.println(speeds);
    SwerveModuleState[] newStates = VisionSimConstants.Kinematics.getValue().toSwerveModuleStates(speeds);
    int constantMultiplier = 1;
    for (int i = 0; i < 4; i++) {
      double distanceError = encoderNoise.nextGaussian(VisionSimConstants.kDistanceEncoderErrorMean,
          VisionSimConstants.kDistanceEncoderStandardDev);
      double angleError = encoderNoise.nextGaussian(VisionSimConstants.kAngleEncoderErrorMean,
          VisionSimConstants.kAngleEncoderStandardDev);
      VisionSimConstants.SwerveModulePositions.getValue()[i].angle = newStates[i].angle
          .plus(new Rotation2d(angleError));
      VisionSimConstants.SwerveModulePositions.getValue()[i].distanceMeters += newStates[i].speedMetersPerSecond
          * timeDelta * constantMultiplier + distanceError;
      System.out.println(VisionSimConstants.SwerveModulePositions.getValue()[i]);
    }
    poseEstimator.get().updateWithTime(Timer.getFPGATimestamp(), gyro.get().toRotation2d(),
        VisionSimConstants.SwerveModulePositions.getValue());
    visionSim.update(simPose);
    // System.out.println("Estimated " +
    // poseEstimator.get().getEstimatedPosition().toString());
    // System.out.println("Actual " + simPose.toPose2d().toString());

    visionSim.getDebugField().getObject("estimate").setPose(poseEstimator.get().getEstimatedPosition());

    if (noiseOverTime.size() % 250 == 0) {
      // System.out.println(noiseOverTime);
      double sum = 0;
      for (double d : noiseOverTime)
        sum += d;
      // System.out.println(sum / (double) noiseOverTime.size());
    }
    lastTimestamp = Timer.getFPGATimestamp();
    System.out.println(poseEstimator.get().getEstimatedPosition());
  }
}
