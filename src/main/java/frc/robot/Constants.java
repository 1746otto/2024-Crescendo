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
    public static final int kSparkBaseID = 2;
    public static final int kIntakeTurnID = 0;
    public static final int kIntakeID = 1;
    public static final double kIntakeSpeed = 0.2;
    public static final double kIntakeStowSpeed = 0.1;
    public static final double kIntakeCurrentLimit = 30;

    // PID constants
    public static final double kP = 0.1;

    // Positions
    public static final double kOriginPosition = 0.0;
    public static final double kOutPosition = 20.0;

    // tolerance
    public static final double kTolerance = 5;
  }
}
