// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.*;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  // float[] states = {
  //   SwerveDrivetrain.SwerveDriveState.getModule(0).speedMetersPerSecond
  // };

  // modulestate = {
  //   m_moduleSpeeds[0], m_moduleDirections[0],
  //   m_moduleSpeeds[1], m_moduleDirections[1],
  //   m_moduleSpeeds[2], m_moduleDirections[2],
  //   m_moduleSpeeds[3], m_moduleDirections[3]
  // };

  // modulestate[0] = m_moduleSpeeds[0];
  // modulestate[1] = m_moduleDirections[0];
  // modulestate[2] = m_moduleSpeeds[1];
  // modulestate[3] = m_moduleDirections[1];
  // modulestate[4] = m_moduleSpeeds[2];
  // modulestate[5] = m_moduleDirections[2];
  // modulestate[6] = m_moduleSpeeds[3];
  // modulestate[7] = m_moduleDirections[3];

  private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
  private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
      m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
  };

  
  // WPILib
  //StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", states.struct).publish();

  
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      //Logger.recordOutput("MyStates", states);

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start();} // Start logging! No more data receivers, replay sources, or metadata values may be added.
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //publisher.set(states);
    //Logger.recordOutput("MyStates", states); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
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
