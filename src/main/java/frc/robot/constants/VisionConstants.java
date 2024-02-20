package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final double kAngleMargin = Units.degreesToRadians(3.0); // TODO: Tune the angle margin for the robot.
    public static final double kAngularPerpendicularityCutoff = Units.degreesToRadians(7.0); // Perpendicularity as in the direction facing and the plane of the tag. Maybe parallel rename to parallel cutoff? TODO: Tune the perpendicularity cutoff for this robot.
    public static final double kDistanceCutoff = Units.feetToMeters(15.0); // TODO: Tune the distance cutoff for this robot.
    public static final double kAmbiguityCutoff = 0.2; // TODO: Tune the ambiguity cutoff for this robot. It is probably fine, but we need to test it.
    public static final String[] kCameraNames = {
        "frontCamera",
        "backCamera"
    };
    public static final int kCameraCount = kCameraNames.length;
    public static final Transform3d[] kCameraTransforms = {
        new Transform3d(new Pose3d(), new Pose3d(0.5, 0, 0.5, new Rotation3d())), 
        new Transform3d(new Pose3d(), new Pose3d(-0.5, 0, 0.5, new Rotation3d()))
    }; // TODO: Get the actual transforms for the cameras.
    public static final Vector<N3> kVisionStdDeviations = VecBuilder.fill(0.9, 0.9, 0.9); // TODO: Tune these values for our robot.
    
}