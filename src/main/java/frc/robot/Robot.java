// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_robotContainer.vision.startThread();
    
    SmartDashboard.putData(m_robotContainer.autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if (m_robotContainer.autoChooser.getSelected() != m_robotContainer.currentAuton) {
      m_robotContainer.autonCommand = m_robotContainer.drivetrain.getAutoPath(m_robotContainer.autoChooser.getSelected());
      m_robotContainer.currentAuton = m_robotContainer.autoChooser.getSelected();
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
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
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    
  
  if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        m_robotContainer.temp = -1;
        VisionConstants.kSpeakerId = 7;
        VisionConstants.kSpeakerPose = new Translation2d(FieldConstants.blueSpeakerX, FieldConstants.blueSpeakerY);
      } else {
        m_robotContainer.temp = 1;
        VisionConstants.kSpeakerId = 3;
        VisionConstants.kSpeakerPose = new Translation2d(FieldConstants.redSpeakerX, FieldConstants.redSpeakerY);
      } 
    } else {
      m_robotContainer.temp = -1;
      VisionConstants.kSpeakerId = 7;
      VisionConstants.kSpeakerPose = new Translation2d(FieldConstants.blueSpeakerX, FieldConstants.blueSpeakerY);
    }
    m_robotContainer.setTeleopInitState();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
