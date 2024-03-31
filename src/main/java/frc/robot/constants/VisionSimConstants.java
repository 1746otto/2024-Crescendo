package frc.robot.constants;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class VisionSimConstants {
    static {
        SwerveModuleConstants = TunerConstants.createSimDrivetrain();
        SwerveDrivetrainConstants = new VisionSimConstants.Lazy<SwerveDrivetrainConstants>(
            () -> TunerConstants.SIM_DRIVETRAIN_CONSTANTS);
        initializeValues();
    }


    public static double kDistanceEncoderErrorMean = 0;
    public static double kDistanceEncoderStandardDev = .012;

    public static double kAngleEncoderErrorMean = 0;
    public static double kAngleEncoderStandardDev = Math.toRadians(0.6);

    public static class Lazy<T> {
        private Optional<T> value = Optional.empty();
        private Supplier<T> valueSupplier;

        public Lazy(Supplier<T> valueSupplier) {
            this.valueSupplier = valueSupplier;
        }

        public T getValue() {
            if (value.isEmpty()) {
                value = Optional.of(valueSupplier.get());
            }
            return value.get();
        }
    }

    public static SwerveModuleConstants[] SwerveModuleConstants;

    public static Lazy<SwerveDrivetrainConstants> SwerveDrivetrainConstants;

    public static Lazy<SwerveModule[]> SwerveModules;
    public static Lazy<SwerveModulePosition[]> SwerveModulePositions;
    public static Lazy<Translation2d[]> SwerveModuleLocations;
    public static Lazy<SwerveDriveKinematics> Kinematics = new Lazy<SwerveDriveKinematics>(
            () -> new SwerveDriveKinematics(VisionSimConstants.SwerveModuleLocations.getValue()));

    public static Lazy<SimCameraProperties> CameraProperties = new Lazy<SimCameraProperties>(() -> {
        SimCameraProperties cameraProperties = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProperties.setCalibration(1200, 1600, Rotation2d.fromDegrees(94));
        // Approximate detection noise with average and standard deviation error in
        // pixels.
        cameraProperties.setCalibError(3.25, 0.25);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        cameraProperties.setFPS(24);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProperties.setAvgLatencyMs(39);
        cameraProperties.setLatencyStdDevMs(2);
        return cameraProperties;
    });

    private static void initializeValues() {
        SwerveModule[] modules = new SwerveModule[4];
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        Translation2d[] moduleLocations = new Translation2d[4];

        int iteration = 0;
        for (SwerveModuleConstants module : SwerveModuleConstants) {
            modules[iteration] = new SwerveModule(module,
                    VisionSimConstants.SwerveDrivetrainConstants.getValue().CANbusName);
            moduleLocations[iteration] = new Translation2d(module.LocationX, module.LocationY);
            modulePositions[iteration] = modules[iteration].getPosition(true);

            iteration++;

        }
        SwerveModules = new Lazy<SwerveModule[]>(() -> modules);
        SwerveModulePositions = new Lazy<SwerveModulePosition[]>(() -> modulePositions);
        SwerveModuleLocations = new Lazy<Translation2d[]>(() -> moduleLocations);
    }
}
