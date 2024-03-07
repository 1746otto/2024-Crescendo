package frc.robot.constants;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

public class AutoConstants {
  public static final PathConstraints dynamicPlanningPathConstraints = new PathConstraints(7, 2.5, 1.5 * Math.PI, 1.5 * Math.PI); // Rotationals could be higher
  public static final PIDConstants translationConstants = new PIDConstants(2.8, 0, 0);
  public static final PIDConstants rotationalConstants = new PIDConstants( 2.8, 0, 0);
}
