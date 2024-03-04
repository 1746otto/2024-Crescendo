package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.Constants.ShooterWristConstants;

public class AmpPosition extends Command {

    private ShooterPivotSubsystem m_shooterPivot;



    public AmpPosition(ShooterPivotSubsystem shooterPivot) {
        m_shooterPivot = shooterPivot;
        addRequirements(shooterPivot);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterPivot.goToAmpPose();
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
    return m_shooterPivot.atRequest(ShooterWristConstants.kampPos);
  }
}


