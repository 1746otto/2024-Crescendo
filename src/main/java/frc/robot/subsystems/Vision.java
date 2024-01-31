package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
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
    boolean continueLoop;

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
                
                SmartDashboard.putNumber("getTimestampSeconds", lastResult.getTimestampSeconds());
                SmartDashboard.putNumber("FPGA Timestamp - latency", Timer.getFPGATimestamp() - lastResult.getLatencyMillis() / 1000.0);
                
                for (PhotonTrackedTarget target : lastResult.targets) {
                    if (target.getFiducialId() > 0 || target.getFiducialId() <= 16)
                        continue;
                    tempPose = field.getTagPose(target.getFiducialId()).get()
                        .transformBy(target.getBestCameraToTarget().inverse())
                        .transformBy(VisionConstants.camera1Transform.inverse());
                    if (Math.abs(tempPose.getRotation().getZ() - swerve.getRotation3d().getZ()) < VisionConstants.kAngleMargin && target.getBestCameraToTarget().getTranslation().getNorm() < VisionConstants.kDistanceCutoff) {
                        while (continueLoop == true) {
                            // Rohan wouldn't let me use for loop :(
                            continueLoop = false;
                            try {
                                swerve.addVisionMeasurement(tempPose.toPose2d(), lastResult.getTimestampSeconds());
                            } catch (Exception e) {
                                continueLoop = true;
                            }
                        }
                        continue;
                    }
                    tempPose = field.getTagPose(target.getFiducialId()).get()
                        .transformBy(target.getAlternateCameraToTarget().inverse())
                        .transformBy(VisionConstants.camera1Transform.inverse());
                    if (Math.abs(tempPose.getRotation().getZ() - swerve.getRotation3d().getZ()) < VisionConstants.kAngleMargin && target.getAlternateCameraToTarget().getTranslation().getNorm() < VisionConstants.kDistanceCutoff) {
                        while (continueLoop == true) {
                            continueLoop = false;
                            try {
                                swerve.addVisionMeasurement(tempPose.toPose2d(), lastResult.getTimestampSeconds());
                            } catch (Exception e) {
                                continueLoop = true;
                            }
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
