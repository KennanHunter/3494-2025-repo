package frc.robot.subsystems.superstructure.Elevator;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // IO layer
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Distance ELEVATOR_ACCEPTABLE_HEIGHT_ERROR = Meters.of(0.1);

  private ElevatorState target;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public ElevatorState getState() {
    return new ElevatorState(inputs.currentHeight, IdleMode.kBrake);
  }

  public void setTargetState(ElevatorState targetState) {
    this.target = targetState;

    io.runElevatorHeight(targetState.height());
  }

  public ElevatorSensorState getSensorState() {
    return inputs.sensorState;
  }

  public boolean isAtTarget() {
    if (this.target == null) return false;

    return this.target.height().minus(getState().height()).abs(Meters)
        <= ELEVATOR_ACCEPTABLE_HEIGHT_ERROR.in(Meters);
  }
}
