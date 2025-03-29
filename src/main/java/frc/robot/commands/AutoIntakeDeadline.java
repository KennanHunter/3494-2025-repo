package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure.Intake;

public class AutoIntakeDeadline extends Command {
  Intake intake;

  public AutoIntakeDeadline(Intake intake) {
    this.intake = intake;
  }

  @Override
  public boolean isFinished() {
    if (intake.hasCoral()) return true;
    return false;
  }
}
