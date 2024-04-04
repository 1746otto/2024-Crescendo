package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BackpackRollerConstants;
import frc.robot.Constants.BackpackWristConstants;

public class BackpackRollerSubsystem extends SubsystemBase{

    CANSparkMax roller;

    public BackpackRollerSubsystem() {
        roller = new CANSparkMax(BackpackRollerConstants.kBackpackRollerID, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        roller.set(speed);
    }

    public Command shootCommand() { //bad naming lol
        return runOnce(() -> setSpeed(BackpackRollerConstants.kOuttake));
    }
}
