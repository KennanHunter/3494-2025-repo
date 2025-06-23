package frc.robot.subsystems.superstructure.Elevator;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // IO layer
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Distance ELEVATOR_ACCEPTABLE_HEIGHT_ERROR = Meters.of(0.1);

  // TODO: Try on real robot, might want to be debouncing both rising and falling
  public Debouncer boundResetFilter = new Debouncer(0.1, DebounceType.kRising);
  private boolean lastDebouncedState = false;

  private ElevatorState target;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Get current debounced state
    boolean currentDebouncedState =
        boundResetFilter.calculate(inputs.sensorState == ElevatorSensorState.BOTTOM);

    boolean shouldResetDebounced = currentDebouncedState && !lastDebouncedState;

    lastDebouncedState = currentDebouncedState;

    Logger.recordOutput("Elevator/ShouldResetDebounced", shouldResetDebounced);
    Logger.recordOutput("Elevator/DebouncedState", currentDebouncedState);

    if (shouldResetDebounced) {
      io.resetHeight(Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT);
    }
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
