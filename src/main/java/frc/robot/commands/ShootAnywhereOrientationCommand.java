package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootAnywhereOrientationCommand extends Command {
    Vision robotVision;
    CommandSwerveDrivetrain drivetrain;
    SwerveRequest.FieldCentric drive;

    double currentYaw;
    double targetYaw = 0.0;
    public ShootAnywhereOrientationCommand(Vision vision, CommandSwerveDrivetrain swerve, SwerveRequest.FieldCentric drive)
    {
        robotVision = vision;
        drivetrain = swerve;
        this.drive = drive;
    }

    public Pose3d getBestRobotPose()
    {
        Pose3d[] poses = robotVision.outputRobotPoseVision();
        Pose3d combinedPose = poses[poses.length - 1];
        return combinedPose;
    }

    public double getCurrentYaw()
    {
        Pose3d pose = getBestRobotPose();
        return pose.getRotation().getZ();
    }

    @Override
    public void execute()
    {
        double turn = -1.0 * getCurrentYaw() * 0.5 * Constants.TeleopSwerveConstants.MaxAngularRateRotPerSec;
        drivetrain.applyRequest(() ->
            drive.withVelocityX(0).
            withVelocityY(0).
            withRotationalRate(turn)
        );
    }

    @Override
    public boolean isFinished()
    {
        return (Math.abs(getCurrentYaw() - targetYaw) <= Units.degreesToRadians(10));
    }


}
