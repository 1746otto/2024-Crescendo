package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * Class for PrimerSubsystem to move game pieces from indexer to a holding space near the shooter.
 */
public class PrimerSubsystem extends SubsystemBase{

  private TalonFX primerRoller;
  private AnalogInput speakerBeamBreak;
  public boolean primerStow;

 

  /**
   * Creates a new PrimerSubsystem with initialized motor controller.
   */
  public PrimerSubsystem() {
    primerRoller = new TalonFX(PrimerConstants.kPrimerRollerMotorID);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(PrimerConstants.kStatorLimit)
      .withSupplyCurrentLimit(PrimerConstants.kSupplyLimit);
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.Slot0 = new Slot0Configs()
      .withKP(PrimerConstants.kVelocityP)
      .withKI(PrimerConstants.kVelocityI)
      .withKD(PrimerConstants.kVelocityD)
      .withKS(PrimerConstants.kVelocityS)
      .withKV(PrimerConstants.kVelocityV)
      .withKA(PrimerConstants.kVelocityA);
    configs.Slot1 = new Slot1Configs()
      .withKP(PrimerConstants.kPositionP)
      .withKI(PrimerConstants.kPositionI)
      .withKD(PrimerConstants.kPositionD)
      .withKS(PrimerConstants.kPositionS)
      .withKV(PrimerConstants.kPositionV)
      .withKA(PrimerConstants.kPositionA);
    
    primerRoller.getConfigurator().apply(configs);

    
    primerRoller.setInverted(false);
    speakerBeamBreak = new AnalogInput(ShooterConstants.kShooterAnalogInputChannel);
    // pidController.setP(PrimerConstants.kP);
    // pidController.setI(PrimerConstants.kI);
    // pidController.setFF(PrimerConstants.kFF);

  }

  /**
   * Sets the priming roller speed for priming the shooting mechanism.
   */
  public void primeNote(double rpm) {
    //pidController.setReference(rpm, ControlType.kVelocity);
  }
  
  public void setSpeed(double speed) {
    primerRoller.set(speed);
  }
  /**
   * Run the motor backwards at a slow speed of kPrimerReverseSpeed.
   */
  
  public void returnNote() {
    primerRoller.set(PrimerConstants.kOuttake);
  }
  public void note() {
    primerRoller.set(PrimerConstants.kIntake);
  }
  /**
   * Stops the primer motor
   */
  public void stop() {
    primerRoller.set(PrimerConstants.kStop);
  }

  /**
   * Checks if the game piece is pinched into the primer at a certain point.
   * @return true or false if game piece is held well in primer.
   */
  public boolean isObjectPinchedInPrimer() {
    return (isPrimerBeamBreakBroken());
  }
  public boolean priming(){
    return (primerRoller.getVelocity().getValueAsDouble() != 0);
  }

  /** Returns the velocity of the motor */
  // public double getVelocity() {
  //   return pidController.getSmartMotionMaxVelocity(PrimerConstants.kPrimerSlotID);
  // }


  public void holdPosition() {
    double tempPosition = primerRoller.getPosition().getValueAsDouble();
    primerRoller.setControl(new PositionVoltage(tempPosition));
  }
  public Command ampCommand() {
    return setSpeedCommand(PrimerConstants.kAmp);
  }

  /**
   * Runs the primer forward until the beambreak is broken.
   * <p> Ends on: 
   * <ul>
   *   <li> Beambreak broken
   * </ul>
   * <p> End Behavior: 
   * <ul>
   *   <li> Whether interrupted or not, sets speed to 0 upon finishing.
   * </ul>
   * @return Command
   */
  public Command intakeCommand() {
    if (Utils.isSimulation()) {
      return new WaitCommand(2.5);
    }
    return setSpeedCommand(PrimerConstants.kIntake).andThen(new WaitUntilCommand(this::isPrimerBeamBreakBroken)).finallyDo(() -> setSpeed(0));
  }

  public Command fastIntakeCommand() {
    return setSpeedCommand(.5).andThen(new WaitUntilCommand(this::isPrimerBeamBreakBroken)).andThen(backupCommand());
  }
  
  public Command setIntakeSpeed() {
    return runOnce(() -> setSpeed(PrimerConstants.kIntake));
  }

  public Command outtakeCommand() {
    return setSpeedCommand(PrimerConstants.kOuttake).finallyDo(() -> setSpeed(0));
  }
  public Command shootCommand() {
    return setSpeedCommand(PrimerConstants.kShoot).finallyDo(() -> setSpeed(0));
  }
  public Command stopCommand() {
    return setSpeedCommand(PrimerConstants.kStop);
  }

  public Command setOuttakeSpeed() {
    return runOnce(() -> setSpeed(PrimerConstants.kOuttake));
  }

  public Command backupCommand() {
    return runOnce(() -> setSpeed(-.1)).andThen(new WaitUntilCommand(() -> isPrimerBeamBreakBroken()).withTimeout(0.15)).finallyDo(() -> setSpeed(0));
  }

  public Command setSpeedCommand(double speed) {
    return runOnce(() -> setSpeed(speed));
  }
  public boolean isPrimerBeamBreakBroken() { //To change 
    return ((Math.floor(speakerBeamBreak.getVoltage()) == 0));
}
  @Override
  public void periodic() {
    //System.out.println(isObjectPinchedInPrimer()); //To change
    SmartDashboard.putBoolean("Beambreak", isPrimerBeamBreakBroken());
    if (primerStow && isPrimerBeamBreakBroken()) {
      //setSpeed(PrimerConstants.kIntake);
    }
  
  }
  
}
