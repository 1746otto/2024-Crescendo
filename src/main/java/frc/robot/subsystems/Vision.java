package frc.robot.subsystems;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    public PhotonCamera[] cameras = new PhotonCamera[VisionConstants.kCameraCount];
    public PhotonPipelineResult[] results = new PhotonPipelineResult[VisionConstants.kCameraCount];
    public Pose3d[] cameraPoses = new Pose3d[VisionConstants.kCameraCount];
    public AprilTagFieldLayout field;

    public Vision() {
        for (int i = 0; i < VisionConstants.kCameraCount; i++) {
            cameras[i] = new PhotonCamera(VisionConstants.kCameraNames[i]);
            cameraPoses[i] = new Pose3d();
        }
        try {
            field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (Exception e) {
            System.out.println("Vission Error Message: " + e.getMessage());
        }
    }

    // Gets latest result [PhotonPipelineResult] from each camera
    private void getResult() {
        for (int i = 0; i < VisionConstants.kCameraCount; i++) {
            results[i] = cameras[i].getLatestResult();
        }
    }

    // Returns robot pose [Pose3d] from the best target for a single camera
    private Pose3d bestTargetToRobotPose(PhotonTrackedTarget target, int cameraNumber) {
        // Check if AprilTag ID is within range
        int tagID = target.getFiducialId();
        if (tagID < VisionConstants.kMinTagID || tagID > VisionConstants.kMaxTagID) {
            return null;
        }
        // Check if target is within range
        double distance = target.getBestCameraToTarget().getTranslation().getNorm();
        if (distance >= VisionConstants.kMaxDistance) {
            return null;
        }
        return field.getTagPose(target.getFiducialId()).get()
        .transformBy(target.getBestCameraToTarget().inverse())
        .transformBy(VisionConstants.kCameraTransforms[cameraNumber].inverse());
    }

    // Returns robot pose [Pose3d] from the multitarget for a single camera
    private Pose3d multiTargetToRobotPose(MultiTargetPNPResult multiTarget, int cameraNumber) {
        // Check if AprilTag ID is within range
        for (int tagID : multiTarget.fiducialIDsUsed) {
            if (tagID < VisionConstants.kMinTagID || tagID > VisionConstants.kMaxTagID) {
                return null;
            }
        }
        // Check if multitarget is available
        if (multiTarget.estimatedPose.isPresent) {
            Translation3d multiTargetTranslation = multiTarget.estimatedPose.best.getTranslation();
            Rotation3d multiTargetRotation = multiTarget.estimatedPose.best.getRotation();
            Pose3d multiTargetPose = new Pose3d(multiTargetTranslation, multiTargetRotation);
            return multiTargetPose.transformBy(VisionConstants.kCameraTransforms[cameraNumber].inverse());
        } else {
            return null;
        }
    }

    // Returns robot pose [Pose3d] from the best targets for all cameras
    private Pose3d combinedBestTargetToRobotPose(PhotonTrackedTarget[] bestTargets) {
        double totalWeight = 0;
        Translation3d weightedTranslationSum = new Translation3d(0, 0, 0);
        double weightedRotXSum = 0;
        double weightedRotYSum = 0;
        double weightedRotZSum = 0;
        for (int i = 0; i < bestTargets.length; i++) {
            PhotonTrackedTarget target = bestTargets[i];
            // Check if target is available
            if (target == null) {
                continue;
            }
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
            double error = getErrorFromDistance(distance);
            double weight = 1.0 / error;
            Pose3d robotPose = bestTargetToRobotPose(target, i);
            if (robotPose == null) {
                continue;
            }
            weightedTranslationSum = weightedTranslationSum.plus(robotPose.getTranslation().times(weight));
            Rotation3d rotation = robotPose.getRotation();
            weightedRotXSum += rotation.getX() * weight;
            weightedRotYSum += rotation.getY() * weight;
            weightedRotZSum += rotation.getZ() * weight;
            totalWeight += weight;
        }
        // Check if no targets are available for all cameras
        if (totalWeight == 0) {
            return null;
        }
        Translation3d avgTranslation = weightedTranslationSum.div(totalWeight);
        Rotation3d avgRotation = new Rotation3d(weightedRotXSum / totalWeight, weightedRotYSum / totalWeight, weightedRotZSum / totalWeight);
        return new Pose3d(avgTranslation, avgRotation);
    }
    
    // Returns expected error [double] as a function of distance
    private double getErrorFromDistance(double distance) {
        double error;
        // Experimentally measured error function of distance
        // error = 0.03439 * distance + 0.5518; // Linear punishes farther distances less
        error = 0.00005093 * distance * distance + 0.02239 * distance + 1.068; // Quadratic punishes farther distances more
        return error;
    }    

    // Returns estimated robot pose [Pose3d[]] 
    public Pose3d[] outputRobotPoseVision() {
        getResult();
        Pose3d[] outputPoses = new Pose3d[2 * VisionConstants.kCameraCount + 1];
        PhotonTrackedTarget[] bestTargets = new PhotonTrackedTarget[VisionConstants.kCameraCount];
        for (int i = 0; i < VisionConstants.kCameraCount; i++) {
            PhotonPipelineResult result = results[i];
            if (result.hasTargets()) {
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                bestTargets[i] = bestTarget;
                Pose3d individualPose = bestTargetToRobotPose(bestTarget, i);
                outputPoses[i] = individualPose;
                MultiTargetPNPResult multiTarget = result.getMultiTagResult();
                Pose3d multiTargetPose = multiTargetToRobotPose(multiTarget, i);
                outputPoses[i + VisionConstants.kCameraCount] = multiTargetPose;
            } else {
                outputPoses[i] = null;
                outputPoses[i + VisionConstants.kCameraCount] = null;
            }
        }
        Pose3d combinedPose = combinedBestTargetToRobotPose(bestTargets);
            outputPoses[2 * VisionConstants.kCameraCount] = combinedPose;
        return outputPoses;
    } 



    // Shoot Anywhere Methods

    // Returns shooter angle [double] as a function of distance
    private double getShooterAngleFromDistance(double distance) {
        distance = Units.metersToInches(distance);
        double angle;
        // Experimentally measured angle function of distance
        angle = -0.1721 * Math.pow(Math.E, -0.01708 * distance) - 0.2843;
        return angle;
    }

    // Returns shooter RPM [double] as a function of distance
    private double getShooterRPMFromDistance(double distance) {
        distance = Units.metersToInches(distance);
        double RPM;
        // Experimentally measured RPM function of distance
        RPM = 0.03923 * Math.pow(distance, 2) - 2.012 * distance + 5157;
        return RPM;
    }

    // Returns combinedPose [Pose2d]
    private Pose2d getBestRobotPose() {
        Pose3d[] poses = outputRobotPoseVision();
        Pose3d combinedPose = poses[poses.length - 1];
        return combinedPose.toPose2d();
    }

    // Returns speaker pose [Pose2d] for alliance
    private Pose2d getSpeakerPose() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                return new Pose2d(FieldConstants.redSpeakerX, FieldConstants.redSpeakerY, new Rotation2d(Units.degreesToRadians(180)));
            } else {
                return new Pose2d(FieldConstants.blueSpeakerX, FieldConstants.blueSpeakerY, new Rotation2d(Units.degreesToRadians(0)));
            }
        } else {
            return new Pose2d(FieldConstants.redSpeakerX, FieldConstants.redSpeakerY, new Rotation2d(Units.degreesToRadians(180)));
        }
    }

    // Returns distance [double] from speaker
    public double getDistanceFromSpeaker() {
        Pose2d robotPose = getBestRobotPose();
        Pose2d speakerPose = getSpeakerPose();

        double deltaX = speakerPose.getX() - robotPose.getX();
        double deltaY = speakerPose.getY() - robotPose.getY();

        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        SmartDashboard.putNumber("Speaker Distance", distance);
        return distance;
    }

    // Returns shooter RPM [double] for speaker shooting
    public double getTargetShooterRPM() {
        double targetRPM = getShooterRPMFromDistance(getDistanceFromSpeaker());
        SmartDashboard.putNumber("target RPM", targetRPM);
        return targetRPM;
    }

    // Returns shooter angle [double] for speaker shooting
    public double getTargetShooterAngle() {
        double targetAngle = getShooterAngleFromDistance(getDistanceFromSpeaker());
        SmartDashboard.putNumber("target Angle", targetAngle);
        return targetAngle;
    }

}