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
        pivot.goToIntakePose(),
        intakeWrist.setIntakePose(), 
        intakeRollers.intakeWithCurrentSensing(),
        intakeWrist.setStowedPose(),
        intakeRollers.hold().until(() -> intakeWrist.isAtReqPosition(IntakeWristConstants.kIntakePosition)),
        new ParallelDeadlineGroup(
          primer.setToForward(),
          intakeRollers.outtakeWithCurrentSensing(),
          indexer.index()
        ),
        pivot.goToAmpPose()
        );
    }
}
