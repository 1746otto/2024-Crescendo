package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

public class Vision {
    Thread visionThread;
    PhotonCamera camera1;
    // PhotonCamera camera2;
    PhotonPipelineResult lastResult;
    Pose3d camera1Pose;

    public Vision() {
        camera1 = new PhotonCamera("frontCamera");
        lastResult = new PhotonPipelineResult();

        visionThread.setName("Vision Thread");
        visionThread = new Thread(() -> {
            while (true) {
                getResult();
                for (PhotonTrackedTarget target : lastResult.targets) {
                    
                }
            }
        });
    }

    private void getResult() {
        lastResult = camera1.getLatestResult();
    }
}
