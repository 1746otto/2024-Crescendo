package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.BackpackWristConstants;
import frc.robot.Constants.IntakeRollerConstants;

public class BackpackWristSubsystem extends SubsystemBase{
    
    TalonFX wrist;
    double target;
    
    public BackpackWristSubsystem() {

        wrist = new TalonFX(BackpackWristConstants.kMotorID);
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IntakeRollerConstants.kStatorLimit)
            .withSupplyCurrentLimit(IntakeRollerConstants.kSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wrist.getConfigurator().apply(configs);
    }

    public void wristToReq(double target) {
        wrist.setControl(new PositionVoltage(target));
    }

    public boolean isAtReq(double target) {
        return Math.abs(target - wrist.getPosition().getValueAsDouble()) < BackpackWristConstants.kTolerance;
    }

    public Command runPivot(double target) {
        return runOnce(() -> wristToReq(target)).andThen(new WaitUntilCommand(() -> isAtReq(target)));
    }

    public Command backPackCommand() {
        return runPivot(BackpackWristConstants.kIntake);
    }
    
}
