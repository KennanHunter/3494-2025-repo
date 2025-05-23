package frc.robot.subsystems.superstructure.Elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // IO layer
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

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
    return new ElevatorState(inputs.currentHeight, MetersPerSecond.of(0), IdleMode.kBrake);
  }

  // public void runHeight(Distance height) {
  // Implementation for setting height
  // }

  public ElevatorSensorState getSensorState() {
    return inputs.sensorState;
  }
}
