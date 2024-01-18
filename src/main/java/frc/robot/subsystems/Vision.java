package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision {
    Thread visionThread;
    PhotonCamera camera1;
    // PhotonCamera camera2;
    PhotonPipelineResult lastResult;
}
