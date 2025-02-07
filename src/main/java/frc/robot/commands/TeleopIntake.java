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
    intake.setSpeed(Math.copySign(Math.pow(OI.getIntakePower(), 2), OI.getIntakePower()));
    Logger.recordOutput("Intake/Intake-Power-Command", -1 * Math.pow(OI.getIntakePower(), 2));
      
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
