package frc.robot.constants;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    public static final String[] kCameraNames = {
        "Camera1"
    };
    public static final int kCameraCount = kCameraNames.length;
    // Update with camera transforms
    public static final Transform3d[] kCameraTransforms = {
        new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0))
    };
    public static final int kMaxDistance = 8;
    public static final int kMinTagID = 1;
    public static final int kMaxTagID = 16;
    public static final Vector<N3> kVisionStdDeviations = null;
}
