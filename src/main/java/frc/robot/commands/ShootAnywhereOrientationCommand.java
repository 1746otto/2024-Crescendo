package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Vision;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

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

public class ShootAnywhereOrientationCommand extends Command {
    Vision robotVision;
    CommandSwerveDrivetrain drivetrain;
    SwerveRequest.FieldCentric drive;
    Pose2d redSpeaker;
    double angleToTurn;

    public ShootAnywhereOrientationCommand(Vision vision, CommandSwerveDrivetrain swerve, SwerveRequest.FieldCentric drive) {
        robotVision = vision;
        drivetrain = swerve;
        this.drive = drive;
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

    @Override
    public void execute() {
        Pose2d robotPose = getBestRobotPose();
        Pose2d speakerPose = getSpeakerPose();

        double deltaX = speakerPose.getX() - robotPose.getX();
        double deltaY = speakerPose.getY() - robotPose.getY();

        double targetAngle = Math.atan2(deltaY, deltaX);
        double currentAngle = robotPose.getRotation().getRadians();
        angleToTurn = Math.atan2(Math.sin(targetAngle - currentAngle), Math.cos(targetAngle - currentAngle));
        double turn = angleToTurn * 0.5 * Constants.TeleopSwerveConstants.MaxAngularRateRotPerSec;

        drivetrain.applyRequest(() -> drive.withRotationalRate(turn));
    }

    @Override
    public boolean isFinished() {
        return angleToTurn < VisionConstants.orientationTolerance;
    }
}
