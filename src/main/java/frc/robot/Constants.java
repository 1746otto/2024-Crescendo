// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Constants to initialize controller inputs.
   */
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeRollerConstants
  {
    public static final int kIntakeID = 12;
    public static final double kIntakeSpeed = 0.2;
    public static final double kIntakeStowSpeed = 0.1;
    public static final double kIntakeCurrentLimit = 30;
    
  }

  public static class IntakeWristConstants
  {
    public static final int kIntakeTurnID = 11;
    // PID constants
    public static final double kP = 0.2;
    public static final double kFF = 0.1;

    // Positions
    public static final double kOutakePosition = 17;
    public static final double kOriginPosition = 0;

    // tolerance
    public static final double kTolerance = 5;

  }
  
  

  /**
   * Constants for the IndexerSubsystem.
   * IDs and speeds are included.
   */
  public static class IndexerConstants {
    public static final int kIndexerID = 41;
    public static final double kIndexerSpeed = 0.2;
    public static final double kIndexerRevSpeed = -kIndexerSpeed;
    public static final double kIndexerStopSpeed = 0.0;
    // Constant that sets motor inverted
    public static final boolean kMotorInvert = true;
  }

  /**
   * Constants for the ShooterSubsystem.
   * IDs, speeds, and PID values are included.
   */
  public static class ShooterConstants {
    // speed constants
    public static final double kShooterRollerSpeed = 1.0;
    public static final double kShooterStopSpeed = 0.0;

    // device IDs
    public static final int kShooterTopRollerMotorID = 21;
    public static final int kShooterBottomRollerMotorID = 22;
    public static final int kShooterAnalogInputChannel = 0;

    // PID constants (top shooting roller)
    public static final double topRollerKP = 0.1;
    public static final double topRollerKI = 0;
    public static final double topRollerKD = 0;

  }

  /**
   * Constants for the PrimerSubsystem.
   * IDs, speeds, and current limits are included.
   */
  public static class PrimerConstants {
    // IDs
    public static final int kPrimerRollerMotorID = 31;
    public static final int kPrimerSlotID = 0;
    // Speeds
    public static final double kPrimerRollerSpeed = 0.8;
    public static final double kPrimerReverseSpeed = -0.1;
    public static final double kPrimerStopSpeed = 0.0;
    public static final double kPrimerCurrentLimit = 20;
    public static final double kPrimerPlaceholderSpeed = 0.2;
    // PID values
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kFF = 0.0;
  }// Should go away for final competition code

  public static class LEDConstants {
    public static final int PWMPortLeft = 0;
    public static final int LEDLength = 135;
    public static final int coneHValue = 18;
    public static final int coneSValue = 255;
    public static final int coneVValue = 130;
    public static final int cubeHValue = 134;
    public static final int cubeSValue = 255;
    public static final int cubeVValue = 130;
  }

  public static class ShooterWristConstants {
    public static int ShooterMasterID = 51;
    public static int ShooterSlaveID = 52;
    public static double podiumPos = 0; // To change
    public static double subwooferPos = 0; // To change
    public static double normalPos = 0; // To change
    public static double ampPos = 0;// To change
    public static double tolerance = 0;// To change
    public static double limit = 5.52380952383 / (2 * Math.PI);
    public static double kDt = 0.02;// To change

    // Trapezoidal profiling
    public static double maxVelocity = 1.75;
    public static double maxAcceleration = 0.75;
  }
}
