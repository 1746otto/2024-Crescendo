package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShootAnywhereShooterCommand extends Command {
    Vision robotVision;
    Pose2d redSpeaker;
    double targetAngle;
    double targetRPM;
    double distance;

    public ShootAnywhereShooterCommand(Vision vision) {
        robotVision = vision;
        redSpeaker = new Pose2d(FieldConstants.redSpeakerX, FieldConstants.redSpeakerY, new Rotation2d(Units.degreesToRadians(180)));
    }

    private Pose2d getSpeakerPose() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                return new Pose2d(FieldConstants.redSpeakerX, FieldConstants.redSpeakerY, new Rotation2d(Units.degreesToRadians(180)));
            } else {
                return new Pose2d(FieldConstants.blueSpeakerX, FieldConstants.blueSpeakerY, new Rotation2d(Units.degreesToRadians(0)));
            }
        } else {
            return redSpeaker;
        }
    }

    private Pose2d getBestRobotPose() {
        Pose3d[] poses = robotVision.outputRobotPoseVision();
        Pose3d combinedPose = poses[poses.length - 1];
        return combinedPose.toPose2d();
    }

    private double getShooterAngle(double distance) {
        distance = Units.metersToInches(distance);
        double angle;
        angle = -0.1721 * Math.pow(Math.E, -0.01708 * distance) - 0.2843;
        return angle;
    }

    private double getShooterRPM(double distance) {
        distance = Units.metersToInches(distance);
        double RPM;
        RPM = 0.03923 * Math.pow(distance, 2) - 2.012 * distance + 5157;
        return RPM;
    }

    @Override
    public void execute() {
        Pose2d robotPose = getBestRobotPose();
        Pose2d speakerPose = getSpeakerPose();

        double deltaX = speakerPose.getX() - robotPose.getX();
        double deltaY = speakerPose.getY() - robotPose.getY();

        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

    }
}
