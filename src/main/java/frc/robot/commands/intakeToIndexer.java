package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PrimerSubsystem;

public class intakeToIndexer extends Command {
    private IntakeRollerSubsystem m_intakeRollers;
    private IntakeWristSubsystem m_WristSubsystem;

    public intakeToIndexer(IntakeRollerSubsystem intakeRollers, IntakeWristSubsystem wristSubsystem) {
        m_intakeRollers = intakeRollers;
        m_WristSubsystem = wristSubsystem;
        addRequirements(intakeRollers, wristSubsystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_WristSubsystem.setRequest(IntakeWristConstants.kOutakePosition);
    m_intakeRollers.intake(IntakeRollerConstants.kIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


