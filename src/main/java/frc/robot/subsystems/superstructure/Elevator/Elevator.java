package frc.robot.subsystems.superstructure.Elevator;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // IO layer
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ElevatorState targetElevatorState = new ElevatorState(Inches.of(6), null);

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.runElevatorHeight(targetElevatorState.height());

    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public ElevatorState getState() {
    Logger.recordOutput("Height", inputs.currentHeight);

    return new ElevatorState(inputs.currentHeight, IdleMode.kBrake);
  }

  public ElevatorSensorState getSensorState() {
    return inputs.sensorState;
  }
}
