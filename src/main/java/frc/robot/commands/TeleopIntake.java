package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Intake;

public class TeleopIntake extends Command {
  private Intake intake;
  private Arm arm;
  double lastPower;

  public TeleopIntake(Intake intake, Arm arm) {
    this.intake = intake;
    this.arm = arm;
    addRequirements(intake);
    addRequirements(arm);
  }

  @Override
  public void execute() {
    // TODO: might have to invert intake speeds/directions
    double intakePower = Math.copySign(Math.pow(OI.getIntakePower(), 2), OI.getIntakePower());
    if(intakePower == 0){
      intakePower = 0.075* Math.copySign(1 ,lastPower) ;
    }
    else{
      lastPower = intakePower;
    }
    if(arm.getTargetPosition() == Constants.Presets.armOuttakeL1){
      intakePower *= 0.3;
    }
    intake.setSpeed(intakePower);
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
