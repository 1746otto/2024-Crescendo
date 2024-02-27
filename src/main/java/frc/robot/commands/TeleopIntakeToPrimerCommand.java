package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.Constants.IntakeWristConstants;

public class TeleopIntakeToPrimerCommand extends SequentialCommandGroup{
    public TeleopIntakeToPrimerCommand(IntakeRollerSubsystem intakeRollers, IntakeWristSubsystem intakeWrist, 
    ShooterPivotSubsystem pivot, IndexerSubsystem indexer, PrimerSubsystem primer) {
    addCommands(
        pivot.goToNormalPos(),
        intakeWrist.requestWristToIntakePos(), 
        intakeRollers.intakeWithCurrentSensing(),
        intakeWrist.requestWristToOriginPos(),
        intakeRollers.StowCommand().until(() -> intakeWrist.isAtReqPosition(IntakeWristConstants.kIntakePosition)),
        new ParallelDeadlineGroup(
          primer.ForwardCommand(),
          intakeRollers.outtakeWithCurrentSensing(),
          indexer.indexCommand()
        ),
        pivot.goToAmpPose()
        );
    }
}
