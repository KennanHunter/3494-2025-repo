package frc.robot.subsystems.superstructure.GroundIntake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase {
  private final GroundIntakeIO groundIntakeIO;
  private GroundIntakeIOInputsAutoLogged groundIntakeIOInputs =
      new GroundIntakeIOInputsAutoLogged();

  private int filterSamples =
      (int)
          Math.round(
              Constants.GroundIntake.CURRENT_FILTER_LENGTH.getSeconds()
                  / Constants.SIMULATED_LOOP_TIME);

  private LinearFilter frontRollerCurrentFilter = LinearFilter.movingAverage(filterSamples);
  private LinearFilter backRollerCurrentFilter = LinearFilter.movingAverage(filterSamples);

  public GroundIntake(GroundIntakeIO groundIntakeIO) {
    this.groundIntakeIO = groundIntakeIO;

    Logger.recordOutput(
        "GroundIntake/filterDurationMillis",
        Constants.GroundIntake.CURRENT_FILTER_LENGTH.toMillis());
    Logger.recordOutput("GroundIntake/filterSampleCount", filterSamples);
  }

  @Override
  public void periodic() {
    groundIntakeIO.updateInputs(groundIntakeIOInputs);

    Logger.processInputs("GroundIntake", groundIntakeIOInputs);

    double filteredFrontRollerCurrent =
        frontRollerCurrentFilter.calculate(groundIntakeIOInputs.frontRollerCurrent);
    double filteredBackRollerCurrent =
        backRollerCurrentFilter.calculate(groundIntakeIOInputs.backRollerCurrent);

    Logger.recordOutput("GroundIntake/frontRollerFilteredCurrent", filteredFrontRollerCurrent);
    Logger.recordOutput("GroundIntake/frontRollerFilteredCurrent", filteredBackRollerCurrent);
  }
}
