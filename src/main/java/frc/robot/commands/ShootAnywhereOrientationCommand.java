package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    @Override
    public void execute() {
        Pose2d robotPose = robotVision.getBestRobotPose();
        if (robotPose != null) {
            Pose2d speakerPose = robotVision.getSpeakerPose();

            double deltaX = speakerPose.getX() - robotPose.getX();
            double deltaY = speakerPose.getY() - robotPose.getY();

            double targetAngle = Math.atan2(deltaY, deltaX);
            double currentAngle = robotPose.getRotation().getRadians();
            angleToTurn = Math.atan2(Math.sin(targetAngle - currentAngle), Math.cos(targetAngle - currentAngle));
            double turn = angleToTurn * 0.5 * Constants.TeleopSwerveConstants.MaxAngularRateRotPerSec;

            SmartDashboard.putNumber("turn effort", turn);

            drivetrain.applyRequest(() -> drive.withRotationalRate(turn));
        }
    }

    @Override
    public boolean isFinished() {
        return angleToTurn < VisionConstants.orientationTolerance;
    }
}
