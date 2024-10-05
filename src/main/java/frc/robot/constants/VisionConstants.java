package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class VisionConstants {
    public static final String[] kCameraNames = {
        "Camera1"
    };
    public static final int kCameraCount = kCameraNames.length;
    // Update with camera transforms
    public static final Transform3d[] kCameraTransforms = {
        new Transform3d(new Translation3d(Units.inchesToMeters(12.61043), Units.inchesToMeters(-11.187369), Units.inchesToMeters(6.916453)), new Rotation3d(0, Units.degreesToRadians(-40), Units.degreesToRadians(15)))
    };
    public static final int kMaxDistance = 8;
    public static final int kMinTagID = 1;
    public static final int kMaxTagID = 16;
    public static final Vector<N3> kVisionStdDeviations = VecBuilder.fill(3.3, 3.3, Double.MAX_VALUE);
    public static final double orientationTolerance = 0.1;
}
