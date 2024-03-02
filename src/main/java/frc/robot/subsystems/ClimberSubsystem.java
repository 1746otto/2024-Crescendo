// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private CANSparkMax climberLeader;
  private CANSparkMax climberFollower;
  public final DigitalInput topLimitSwitch = new DigitalInput(ClimberConstants.kLimitSwitch);
  private SparkLimitSwitch m_forwardLimit;
  public ClimberSubsystem() {
    climberLeader = new CANSparkMax(ClimberConstants.kleaderCANid, MotorType.kBrushless);
    climberFollower = new CANSparkMax(ClimberConstants.kfollowerCANid, MotorType.kBrushless);
    m_forwardLimit = climberLeader.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    climberLeader.follow(climberFollower, true);
  }
    
  /**
   * Example command factory method.
   *
   * @return a command
   */
 

  public void setSpeed (double speed) {
    climberLeader.set(speed);
  }

  public Command downCommand() {
        return setSpeedCommand(ClimberConstants.kForward);
    }
    public Command reverseCommand() {
        return setSpeedCommand(ClimberConstants.kReverse);
    }
    public Command stopCommand() {
        return setSpeedCommand(ClimberConstants.kStop);
    }

    public Command setSpeedCommand(double speed) {
      return run(() -> setSpeed(speed)).until(() -> m_forwardLimit.isLimitSwitchEnabled());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
