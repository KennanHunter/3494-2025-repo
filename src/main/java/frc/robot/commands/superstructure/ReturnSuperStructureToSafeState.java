package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.KnownState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ReturnSuperStructureToSafeState extends Command {
  private final SuperStructure superStructure;

  public ReturnSuperStructureToSafeState(SuperStructure superStructure) {
    this.superStructure = superStructure;

    addRequirements(superStructure);
  }

  @Override
  public void execute() {
    superStructure.setTargetKnownState(KnownState.SafeReset);
  }

  @Override
  public boolean isFinished() {
    return superStructure.isAtTarget();
  }
}
