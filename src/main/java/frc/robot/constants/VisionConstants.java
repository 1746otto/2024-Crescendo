package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public static final double kAngleMargin = 3.0;
    public static final double kDistanceCutoff = 15.0;
    public static final double kAmbiguityCutoff = 0.2;
    public static final Transform3d camera1Transform = new Transform3d(new Pose3d(), new Pose3d(0.5, 0, 0.5, new Rotation3d()));
    
}
