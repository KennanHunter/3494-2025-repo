package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.SuperStructure.Intake;

public class TeleopIntake extends Command {
  private Intake intake;

  public TeleopIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    // TODO: might have to invert intake speeds/directions
    if (OI.intakeIn() > 0) {
      Logger.recordOutput("Intake/Intake-Power-Command", Math.pow(OI.intakeIn(), 2));
      intake.setSpeed(Math.pow(OI.intakeIn(), 2));
    } else if (OI.intakeOut() > 0) {
      Logger.recordOutput("Intake/Intake-Power-Command", -1 * Math.pow(OI.intakeOut(), 2));
      intake.setSpeed(-1 * Math.pow(OI.intakeOut(), 2));
    } else {
      Logger.recordOutput("Intake/Intake-Power-Command", 0.0);
      intake.setSpeed(0);
    }
  }

  public boolean isFinished() {
    return false;
  }

  public void end() {
    intake.setSpeed(0);
  }

  public void interrupted() {
    end();
  }
}
