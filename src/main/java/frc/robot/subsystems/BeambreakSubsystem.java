package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.AnalogInput;

public class BeambreakSubsystem {
    /** Analog input for detecting beam breaks. */
  private AnalogInput primerBeambreak;

    public BeambreakSubsystem() {
        //primerBeambreak = new AnalogInput(ShooterConstants.kShooterAnalogInputChannel);
    }

    /**
   * Returns a BooleanSupplier representing if the beambreak has been broken or not.
   */
    public BooleanSupplier isBeamBreakBroken() {
        return () -> ((Math.floor(primerBeambreak.getVoltage()) > 0));
    }
}
