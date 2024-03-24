package frc.robot.subsystems;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.VisionConstants;

public class VisionSim {
    Thread visionThread;
    private VisionSystemSim visionSystemSim;
    PhotonCamera[] cameras = new PhotonCamera[VisionConstants.kCameraCount];
    public volatile PhotonPipelineResult[] lastResults = new PhotonPipelineResult[VisionConstants.kCameraCount];
    public volatile Pose3d[] cameraPoses = new Pose3d[VisionConstants.kCameraCount];
    public volatile Pose3d robotPose; // Might use this in other filter methods later
    AprilTagFieldLayout field;
    private SwerveDrivePoseEstimator poseEstimator;
    private Supplier<Rotation3d> gyro;
    volatile boolean continueLoop;
    int speakerID = 7; 
    String tags = new String();


    public VisionSim(SwerveDrivePoseEstimator poseEstimator, Supplier<Rotation3d> gyro, VisionSystemSim visionSim, SimCameraProperties cameraProperties) {
        visionSystemSim = visionSim;
        for (int i = 0; i < VisionConstants.kCameraCount; i++) {
            cameras[i] = new PhotonCamera(VisionConstants.kCameraNames[i]);
            cameraPoses[i] = new Pose3d();
            PhotonCameraSim cameraSim = new PhotonCameraSim(cameras[i], cameraProperties);
            visionSim.addCamera(cameraSim, VisionConstants.kCameraTransforms[i]);
        }

        getResult();

        this.gyro = gyro;
        this.poseEstimator = poseEstimator;
        this.poseEstimator.setVisionMeasurementStdDevs(VisionConstants.kVisionStdDeviations);

        try {
            field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        }
        catch (Exception e) {
            SmartDashboard.putString("Vision Error Message", e.getMessage());
        }

        visionThread = new Thread(() -> {
            while (true) {
                getResult();
                
                filter4();
            }
        });

        if (DriverStation.getAlliance().isPresent())
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                speakerID = 7;
            }
            else {
                speakerID = 4;
            }

        visionThread.setName("Vision Thread");

        // visionThread.start();
    }

    public void stopThread() {
        visionThread.interrupt();
    }

    public void startThread() {
        visionThread.start();
    }

    public boolean isRunning() {
        return visionThread.isAlive();
    }

    private void getResult() {
        for (int i = 0; i < VisionConstants.kCameraCount; i++) {
            lastResults[i] = cameras[i].getLatestResult();
        }
    }

    private Pose3d bestTargetToRobotPose(PhotonTrackedTarget target, int cameraNumber) {
        return field.getTagPose(target.getFiducialId()).get()
            .transformBy(target.getBestCameraToTarget().inverse())
            .transformBy(VisionConstants.kCameraTransforms[cameraNumber].inverse());
    }

    private Pose3d alternateTargetToRobotPose(PhotonTrackedTarget target, int cameraNumber) {
        return field.getTagPose(target.getFiducialId()).get()
            .transformBy(target.getBestCameraToTarget().inverse())
            .transformBy(VisionConstants.kCameraTransforms[cameraNumber].inverse());
    }


    /**
     * Was it really worth all that effort?
     * Filter methods used:
     * - ID: If the Fiducial ID isn't used on the field the target is discarded.
     * - Distance: Distant tags are discarded due to degradation of the tag resolution.
     * - Ambiguity: Tags with greater that 0.2 pose ambiguity are rejected.
     *   The best pose transform is always used. No alternates are considered.
     */
    private void filter4() {
        for (int i = 0; i < VisionConstants.kCameraCount; i++) {

            SmartDashboard.putNumber("getTimestampSeconds", lastResults[i].getTimestampSeconds());
            SmartDashboard.putNumber("FPGA Timestamp - latency", Timer.getFPGATimestamp() - lastResults[i].getLatencyMillis() / 1000.0);

            for (PhotonTrackedTarget target : lastResults[i].targets) {

                if (target.getFiducialId() > 16 || target.getFiducialId() < 1 || target.getPoseAmbiguity() > VisionConstants.kAmbiguityCutoff)
                    continue;
                
                // Transforms to the pose of the camera, not the robot.
                Pose3d tempPose = bestTargetToRobotPose(target, i);

                // visionSystemSim.getDebugField().getObject("tempPose").setPose(tempPose.toPose2d());
                
                /*
                 * The Math.abs on the raw z position is only necessary if we don't know whether we are above or below the AprilTag.
                 * Assuming I have written this correctly, the Z component of the best pose from the camera perspectective should be
                 * reflected accross the plane where the Z is equal to the tag height. Then the distance from 0 is compared and
                 * depending on which is smaller the best or alternate tag transform is chosen.
                 */
                System.out.println(target.getBestCameraToTarget().getTranslation().getNorm() );
                if (target.getBestCameraToTarget().getTranslation().getNorm() < VisionConstants.kDistanceCutoff) {
                
                    SmartDashboard.putString(VisionConstants.kCameraNames[i] + " pose", tempPose.toString());

                    // This must be here in order to try until the swerve drive unlocks the pose estimator.
                    do {
                        // Rohan wouldn't let me use for loop :(
                        continueLoop = false;
                        try {
                            System.out.println("adding measurement!");
                            poseEstimator.addVisionMeasurement(tempPose.toPose2d(), lastResults[i].getTimestampSeconds());
                        } catch (Exception e) {
                            continueLoop = true;
                        }
                    } while (continueLoop == true);
                }
            }
        }
    }
}
