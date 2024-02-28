package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IndexerConstants;

/**
 * Class for IndexerSubsytem to take game pieces from the intake and move them to the primer.
 */
public class IndexerSubsystem extends SubsystemBase {
    private CANSparkMax indexerMotor;
    /**
     * Initialize the motor controller to invertly run the indexer.
     */
    public IndexerSubsystem() {
        indexerMotor = new CANSparkMax(IndexerConstants.kIndexerID, MotorType.kBrushless);
        indexerMotor.setInverted(IndexerConstants.kMotorInvert);
    }

   
    public void setSpeed(double speed) {
        indexerMotor.set(speed);
    }


    public Command forwardCommand() {
        return run(() -> setSpeed(IndexerConstants.kForward));
    }
    public Command reverseCommand() {
        return run(() -> setSpeed(IndexerConstants.kReverse));
    }
    public Command stopCommand() {
        return run(() -> setSpeed(IndexerConstants.kStop));
    }
    

    public Command setSpeedCommand(double speed) {
        return runOnce(() -> setSpeed(speed));
    }
}