package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.*;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class podiumPosition extends Command {
    private ShooterPivotSubsystem m_PivotSubsystem;

    public podiumPosition(ShooterPivotSubsystem pivotSubsystem) {
        m_PivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PivotSubsystem.runPivot(ShooterWristConstants.kPodiumPos);
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
    return m_PivotSubsystem.atPosition(ShooterWristConstants.kPodiumPos);
  }
}


