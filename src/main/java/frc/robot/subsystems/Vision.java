package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.constants.VisionConstants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

public class Vision {
    Thread visionThread;
    PhotonCamera camera1;
    // PhotonCamera camera2;
    PhotonPipelineResult lastResult;
    Pose3d camera1Pose;
    AprilTagFieldLayout field;
    Pose3d tempPose;
    CommandSwerveDrivetrain swerve;

    public Vision(CommandSwerveDrivetrain swerveDrive) {
        camera1 = new PhotonCamera("frontCamera");
        lastResult = new PhotonPipelineResult();
        swerve = swerveDrive;
        try {
            field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        }
        catch (Exception e) {
            SmartDashboard.putString("Vision Error Message", e.getMessage());
        }

        visionThread.setName("Vision Thread");
        visionThread = new Thread(() -> {
            while (true) {
                getResult();
                for (PhotonTrackedTarget target : lastResult.targets) {
                    if (target.getFiducialId() > 0 || target.getFiducialId() <= 16)
                        continue;
                    if (target.getPoseAmbiguity() > VisionConstants.kAmbiguityCutoff) {
                        tempPose = field.getTagPose(target.getFiducialId()).get()
                            .transformBy(target.getBestCameraToTarget().inverse())
                            .transformBy(VisionConstants.camera1Transform.inverse());
                        if (Math.abs(tempPose.getRotation().getZ() - swerve.getRotation3d().getZ()) < VisionConstants.kAngleMargin && tempPose.getTranslation().getNorm() < VisionConstants.kDistanceCutoff) {
                            swerve.addVisionMeasurement(tempPose.toPose2d(), lastResult.getTimestampSeconds());
                            continue;
                        }
                        tempPose = field.getTagPose(target.getFiducialId()).get()
                            .transformBy(target.getAlternateCameraToTarget().inverse())
                            .transformBy(VisionConstants.camera1Transform.inverse());
                        if (Math.abs(tempPose.getRotation().getZ() - swerve.getRotation3d().getZ()) < VisionConstants.kAngleMargin) {
                            swerve.addVisionMeasurement(tempPose.toPose2d(), lastResult.getTimestampSeconds());
                        }
      
                    }
                }
            }
        });
    }

    private void getResult() {
        lastResult = camera1.getLatestResult();
    }
}
