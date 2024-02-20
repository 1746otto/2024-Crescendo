package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final double kAngleMargin = Units.degreesToRadians(3.0);
    public static final double kAngularPerpendicularityCutoff = Units.degreesToRadians(7.0); // Perpendicularity as in the direction facing and the plane of the tag. Maybe parallel rename to parallel cutoff?
    public static final double kDistanceCutoff = Units.feetToMeters(15.0);
    public static final double kAmbiguityCutoff = 0.2;
    public static final String[] kCameraNames = {
        "frontCamera",
        "backCamera"
    };
    public static final int kCameraCount = kCameraNames.length;
    public static final Transform3d[] kCameraTransforms = {
        new Transform3d(new Pose3d(), new Pose3d(0.5, 0, 0.5, new Rotation3d())), 
        new Transform3d(new Pose3d(), new Pose3d(-0.5, 0, 0.5, new Rotation3d()))
    };
    
}
