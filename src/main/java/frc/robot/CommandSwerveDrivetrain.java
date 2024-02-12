package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

//     DoubleSupplier frontLeftVelocityDS = () -> getState().ModuleStates[0].speedMetersPerSecond / (2 * 40);

//     DoubleSupplier frontRightAngleDS = () -> getState().ModuleStates[1].angle.getDegrees();
// DoubleSupplier frontRightVelocityDS = () -> getState().ModuleStates[1].speedMetersPerSecond / (2 * 40);

public NetworkTable driveTrainBuild = inst.getTable("testFr");

    private final DoublePublisher frontLeftAngle = driveTrainBuild.getDoubleTopic("frontLeftAngle").publish();
    private final DoublePublisher frontLeftVelocity = driveTrainBuild.getDoubleTopic("frontLeftVelocity").publish();
    
    private final DoublePublisher frontRightAngle = driveTrainBuild.getDoubleTopic("frontRightAngle").publish();
    private final DoublePublisher frontRightVelocity = driveTrainBuild.getDoubleTopic("frontRightVelocity").publish();
    
    private final DoublePublisher backLeftAngle = driveTrainBuild.getDoubleTopic("backLeftAngle").publish();
    private final DoublePublisher backLeftVelocity = driveTrainBuild.getDoubleTopic("backLeftVelocity").publish();

    private final DoublePublisher backRightAngle = driveTrainBuild.getDoubleTopic("backRightAngle").publish();
    private final DoublePublisher backRightVelocity = driveTrainBuild.getDoubleTopic("backRightVelocity").publish();

    private final DoublePublisher robotAngle = driveTrainBuild.getDoubleTopic("robotAngle").publish();



    // backLeftAngle.set(getState().ModuleStates[2].angle.getDegrees());
    // backLeftVelocity.set(getState().ModuleStates[2].speedMetersPerSecond / (2 * 40));

    // backRightAngle.set(getState().ModuleStates[3].angle.getDegrees());
    // backRightVelocity.set(getState().ModuleStates[3].speedMetersPerSecond / (2 * 40));

    // robotAngle.set(getPigeon2().getRotation2d().getDegrees());

    // private DoubleSupplier FRONTLEFTTANGLE;
    // private double idk;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData("Swerve Drive", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            driveTrainBuilder = builder;



            driveTrainBuilder.setSmartDashboardType("SwerveDrive");

            
            driveTrainBuilder.addDoubleProperty("Front Left Angle", null, frontLeftAngle);
            driveTrainBuilder.addDoubleProperty("Front Left Velocity", (DoubleSupplier) frontLeftVelocity, null);

            // driveTrainBuilder.addDoubleProperty("Front Right Angle", frontRightAngleDS, null);
            // driveTrainBuilder.addDoubleProperty("Front Right Velocity", frontRightVelocityDS, frontRightVelocity);

            driveTrainBuilder.addDoubleProperty("Back Left Angle", (DoubleSupplier) backLeftAngle, null);
            driveTrainBuilder.addDoubleProperty("Back Left Velocity", (DoubleSupplier) backLeftVelocity, null);

            driveTrainBuilder.addDoubleProperty("Back Right Angle", (DoubleSupplier) backRightAngle, null);
            driveTrainBuilder.addDoubleProperty("Back Right Velocity", (DoubleSupplier) backRightVelocity, null);

            driveTrainBuilder.addDoubleProperty("Robot Angle", (DoubleSupplier) robotAngle, null);
    }
    });
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


            driveTrainBuilder.addDoubleProperty("Front Left Angle", null, frontLeftAngle);
            driveTrainBuilder.addDoubleProperty("Front Left Velocity", (DoubleSupplier) frontLeftVelocity, null);

            // driveTrainBuilder.addDoubleProperty("Front Right Angle", frontRightAngleDS, null);
            // driveTrainBuilder.addDoubleProperty("Front Right Velocity", frontRightVelocityDS, frontRightVelocity);

            driveTrainBuilder.addDoubleProperty("Back Left Angle", (DoubleSupplier) backLeftAngle, null);
            driveTrainBuilder.addDoubleProperty("Back Left Velocity", (DoubleSupplier) backLeftVelocity, null);

            driveTrainBuilder.addDoubleProperty("Back Right Angle", (DoubleSupplier) backRightAngle, null);
            driveTrainBuilder.addDoubleProperty("Back Right Velocity", (DoubleSupplier) backRightVelocity, null);

            driveTrainBuilder.addDoubleProperty("Robot Angle", (DoubleSupplier) robotAngle, null);
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
        driveTrainBuilder.update();
        
    //  System.out.println(frontRightAngleDS.getAsDouble());
        
        frontLeftVelocity.set(getState().ModuleStates[0].speedMetersPerSecond / (2 * 40));
    
        frontRightAngle.set(getState().ModuleStates[1].angle.getDegrees());
        frontRightVelocity.set(getState().ModuleStates[1].speedMetersPerSecond / (2 * 40));
    
        backLeftAngle.set(getState().ModuleStates[2].angle.getDegrees());
        backLeftVelocity.set(getState().ModuleStates[2].speedMetersPerSecond / (2 * 40));

        backRightAngle.set(getState().ModuleStates[3].angle.getDegrees());
        backRightVelocity.set(getState().ModuleStates[3].speedMetersPerSecond / (2 * 40));

        robotAngle.set(getPigeon2().getRotation2d().getDegrees());

       // FRONTLEFTTANGLE = () -> getState().ModuleStates[0].angle.getDegrees();
        //idk = MathUtil.inputModulus(getState().ModuleStates[0].angle.getDegrees(), -180, 180);

        //System.out.println(idk);

    //     SmartDashboard.putData("Swerve Drive", new Sendable() {
    //     @Override
    //     public void initSendable(SendableBuilder builder) {
    //         driveTrainBuilder = builder;



    //         driveTrainBuilder.setSmartDashboardType("SwerveDrive");

            
    //         driveTrainBuilder.addDoubleProperty("frontLeftAngle", () -> getState().ModuleStates[0].angle.getDegrees(), null);
    //         driveTrainBuilder.addDoubleProperty("Front Left Velocity", (DoubleSupplier) frontLeftVelocity, null);

    //         driveTrainBuilder.addDoubleProperty("Front Right Angle", (DoubleSupplier) frontRightAngle, null);
    //         driveTrainBuilder.addDoubleProperty("Front Right Velocity", (DoubleSupplier) frontRightVelocity, null);

    //         driveTrainBuilder.addDoubleProperty("Back Left Angle", (DoubleSupplier) backLeftAngle, null);
    //         driveTrainBuilder.addDoubleProperty("Back Left Velocity", (DoubleSupplier) backLeftVelocity, null);

    //         driveTrainBuilder.addDoubleProperty("Back Right Angle", (DoubleSupplier) backRightAngle, null);
    //         driveTrainBuilder.addDoubleProperty("Back Right Velocity", (DoubleSupplier) backRightVelocity, null);

    //         driveTrainBuilder.addDoubleProperty("Robot Angle", (DoubleSupplier) robotAngle, null);
    //         frontLeftAngle.set(1);
    // }
    // });
    }

    // private double getFrontLeftAngle()
    // {
    //     return getState().ModuleStates[0].angle.getDegrees();
    // }

    // private double test(){
    //     System.out.println(getState().ModuleStates[0].angle.getDegrees());
    //     return 2.0;
    // }
}
