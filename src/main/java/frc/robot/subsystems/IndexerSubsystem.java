package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase{
    CANSparkMax indexerMotor;
    public IndexerSubsystem(){
        indexerMotor = new CANSparkMax(IndexerConstants.kIndexerID, MotorType.kBrushless);
        indexerMotor.setInverted(true);
    }

    public void index(){
        indexerMotor.set(IndexerConstants.kIndexerSpeed);
    }

    public void stop(){
        indexerMotor.set(0);
    }
    public void forward(){
        indexerMotor.set(.2);
    }

    public void backward(){
        indexerMotor.set(-.2);
    }
    
    public Command IndexCommand(){
        return new InstantCommand(() -> index());
    }

    public Command StopCommand(){
        return new InstantCommand(() -> stop());
    }
}