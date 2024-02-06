// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class IntakeConstants {
    public static final int kIntakeTurnID = 21;
    public static final int kIntakeID = 11;
    public static final double kIntakeSpeed = 0.2;
    public static final double kIntakeStowSpeed = 0.1;
    public static final double kIntakeCurrentLimit = 20;

    // PID constants
    public static final double kP = 0.1;

    // Positions
    public static final double kOriginPosition = 0.0;
    public static final double kOutPosition = 21.92;

    // tolerance
    public static final double kTolerance = 5;
  }
  
  public static class IndexerConstants {
    public static final int kIndexerID = 12;
    public static final double kIndexerSpeed = 0.2;
  }

  public static class ShooterConstants {
    // speed constants
    public static final double kShooterRollerSpeed = 1;

    // device IDs 
    public static final int kShooterTopRollerMotorID = 31;
    public static final int kShooterBottomRollerMotorID = 32;
    public static final int kShooterAnalogInputChannel = 0;

    // PID constants (top shooting roller)
    public static final double topRollerKP = 0.1;
    public static final double topRollerKI = 0;
    public static final double topRollerKD = 0;

  }
  
  public static class PrimerConstants {
    // IDs
    public static final int kPrimerRollerMotorID = 22;
    //Speeds
    public static final double kPrimerRollerSpeed = 0.4;
    public static final double kPrimerReverseSpeed = -0.1;
    public static final double kPrimerCurrentLimit = 20;
  }
}
