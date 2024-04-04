package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.BackpackRollerConstants;


public class BackpackRollerSubsystem extends SubsystemBase {
    TalonFX rollerMotor;

    VoltageOut voltage = new VoltageOut(0);

    public BackpackRollerSubsystem() {

        rollerMotor = new TalonFX(BackpackRollerConstants.kBackpackRollerID);
        
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(BackpackRollerConstants.kStatorLimit)
            .withSupplyCurrentLimit(BackpackRollerConstants.kSupplyLimit);
            // .withStatorCurrentLimitEnable(true)
            // .withSupplyCurrentLimitEnable(true);

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        rollerMotor.getConfigurator().apply(configs);
    }

    
    public void setSpeed(double speed) {
        rollerMotor.setControl(voltage.withOutput(speed * 12));
    }

    public void stop() {
        rollerMotor.setControl(new NeutralOut());
    }

    public Command holdCommand() {
        return setSpeedCommand(BackpackRollerConstants.kHold);
    }
    public Command setSpeedCommand(double speed){
        return run(() -> setSpeed(speed)); // needs to be an instant command. MUST FIX AFTER COMP
    }

    public Command intakeSpeedCommand() {
        return runOnce(() -> setSpeed(BackpackRollerConstants.kIntake));
    }
    public Command stopCommand() {
        return runOnce(() -> setSpeed(0));
    }

    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("BackPack Roller speed", rollerMotor.getVelocity().getValueAsDouble());
    }
}