// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.VisionSim;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Optional<VisionSim> visionSimSubsystem;
  private Optional<SwerveDrivePoseEstimator> poseEsimator;
  private VisionSystemSim visionSim = new VisionSystemSim("photonSim2");
  private Pose3d simPose = new Pose3d(13, 2.157, 2, new Rotation3d(0, 0, Math.PI));
  private TargetModel model36h11 = TargetModel.kAprilTag36h11;
  private VisionTargetSim visionTargetSim = new VisionTargetSim(new Pose3d(), model36h11);
  private AprilTagFieldLayout simAprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    visionSim.addAprilTags(simAprilTagFieldLayout);
    visionSim.addVisionTargets(visionTargetSim);
    visionSim.resetRobotPose(simPose);
    if (Utils.isSimulation()) {
      SwerveModule[] modules = new SwerveModule[4];
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      Translation2d[] moduleLocations = new Translation2d[4];

      int iteration = 0;
      for (SwerveModuleConstants module : TunerConstants.createSimDrivetrain() ){
        modules[iteration] = new SwerveModule(module, TunerConstants.SIM_DRIVETRAIN_CONSTANTS.CANbusName);
        moduleLocations[iteration] = new Translation2d(module.LocationX, module.LocationY);
        modulePositions[iteration] = modules[iteration].getPosition(true);

        iteration++;
      }
      Supplier<Rotation3d> gyro = () -> simPose.getRotation();
      SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleLocations);
      poseEsimator = Optional.of(new SwerveDrivePoseEstimator(kinematics,
          simPose.toPose2d().getRotation(), modulePositions, simPose.toPose2d()));
      visionSimSubsystem = Optional.of(new VisionSim(poseEsimator.get(), gyro));
    }
    SmartDashboard.putData(m_robotContainer.autoChooser);
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

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
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
    if (DriverStation.getAlliance().isPresent())
      m_robotContainer.temp = (DriverStation.getAlliance().get() == Alliance.Blue) ? -1 : 1;
    else
      m_robotContainer.temp = -1;

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
  public void simulationPeriodic() {
    System.out.println(visionSim.getRobotPose());
    simPose = simPose.transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.toRadians(1))));
    visionSim.update(simPose);
  }
}
