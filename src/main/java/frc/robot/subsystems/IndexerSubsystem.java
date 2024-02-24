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

    /**
     * Indexer is set to run at the value of kIndexerSpeed.
     */
    public void startIndexing() {
        indexerMotor.set(IndexerConstants.kIndexerSpeed);
    }

    /**
     * Indexer is set to stop running.
     */
    public void stopIndexing() {
        indexerMotor.set(IndexerConstants.kIndexerStopSpeed);
    }

    /**
     * Indexer run back at the value of kIndexerSpeed.
     */
    public void indexBackwards() {
        indexerMotor.set(IndexerConstants.kIndexerRevSpeed);
    }



    
    /**
     * Command to start running the indexer using startIndexing().
     * @return A command that starts runs the indexer
     */
    public Command indexCommand() {
        return runOnce(() -> startIndexing());
    }

    /**
     * Command to stop the indexer using stopIndexing().
     * @return A command to stop the indexer
     */
    public Command stopCommand() {
        return runOnce(() -> stopIndexing());
    }
}