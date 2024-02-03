package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private SendableBuilder driveTrainBuilder;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // SmartDashboard.putData("Swerve Drive", new Sendable() {
        // @Override
        // public void initSendable(SendableBuilder builder) {
        //     driveTrainBuilder = builder;
            
        //     driveTrainBuilder.setSmartDashboardType("SwerveDrive");

        //     driveTrainBuilder.addDoubleProperty("Front Left Angle", () -> getState().ModuleStates[0].angle.getDegrees(), null);
        //     driveTrainBuilder.addDoubleProperty("Front Left Velocity", () -> getState().ModuleStates[0].speedMetersPerSecond / (2 * 40), null);

        //     driveTrainBuilder.addDoubleProperty("Front Right Angle", () ->  getState().ModuleStates[1].angle.getDegrees(), null);
        //     driveTrainBuilder.addDoubleProperty("Front Right Velocity", () -> getState().ModuleStates[1].speedMetersPerSecond / (2 * 40), null);

        //     driveTrainBuilder.addDoubleProperty("Back Left Angle", () ->  getState().ModuleStates[2].angle.getDegrees(), null);
        //     driveTrainBuilder.addDoubleProperty("Back Left Velocity", () -> getState().ModuleStates[2].speedMetersPerSecond / (2 * 40), null);

        //     driveTrainBuilder.addDoubleProperty("Back Right Angle", () ->  getState().ModuleStates[3].angle.getDegrees(), null);
        //     driveTrainBuilder.addDoubleProperty("Back Right Velocity", () -> getState().ModuleStates[3].speedMetersPerSecond / (2 * 40), null);

        //     driveTrainBuilder.addDoubleProperty("Robot Angle", () -> getPigeon2().getRotation2d().getDegrees(), null);
        // }
        // });
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData("Swerve Drive", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            driveTrainBuilder = builder;

            driveTrainBuilder.setSmartDashboardType("SwerveDrive");

            driveTrainBuilder.addDoubleProperty("Front Left Angle", () -> getState().ModuleStates[0].angle.getDegrees(), null);
            driveTrainBuilder.addDoubleProperty("Front Left Velocity", () -> getState().ModuleStates[0].speedMetersPerSecond / (2 * 40), null);

            driveTrainBuilder.addDoubleProperty("Front Right Angle", () ->  getState().ModuleStates[1].angle.getDegrees(), null);
            driveTrainBuilder.addDoubleProperty("Front Right Velocity", () -> getState().ModuleStates[1].speedMetersPerSecond / (2 * 40), null);

            driveTrainBuilder.addDoubleProperty("Back Left Angle", () ->  getState().ModuleStates[2].angle.getDegrees(), null);
            driveTrainBuilder.addDoubleProperty("Back Left Velocity", () -> getState().ModuleStates[2].speedMetersPerSecond / (2 * 40), null);

            driveTrainBuilder.addDoubleProperty("Back Right Angle", () ->  getState().ModuleStates[3].angle.getDegrees(), null);
            driveTrainBuilder.addDoubleProperty("Back Right Velocity", () -> getState().ModuleStates[3].speedMetersPerSecond / (2 * 40), null);

            driveTrainBuilder.addDoubleProperty("Robot Angle", () -> getPigeon2().getRotation2d().getDegrees(), null);
    }
    });
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        //driveTrainBuilder.update();

        
    }
}
