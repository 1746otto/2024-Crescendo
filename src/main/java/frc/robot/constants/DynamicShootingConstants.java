package frc.robot.constants;

import java.util.HashMap;
import java.util.Map;

import org.opencv.core.Mat.Tuple3;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DynamicShootingConstants {

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kMaxAngularVelocity = 4; // Radians per second
  public static final double kMaxAngularAcceleration = 4; // Radians per second squared

  public static final Translation2d blueSpeakerCoordinates = new Translation2d(0, 3);
  public static final Translation2d redSpeakerCoordinates = new Translation2d(17, 3);
  
  public static final double mapStepSize = 0.3;
  public static final Map<Double, Pair<Double, Double>> distanceMap; //(Tuple3<Double>[]) new Object[] {new Tuple3<Double>(3.0, 3.0, 3.0)};
  static {
    Map<Double, Pair<Double, Double>> tempMap = new HashMap<Double, Pair<Double, Double>>();
    tempMap.put(3.0, new Pair<>(4.0, 4.0));
    distanceMap = tempMap;
  };
}
