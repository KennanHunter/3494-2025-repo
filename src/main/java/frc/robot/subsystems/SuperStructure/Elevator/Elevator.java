package frc.robot.subsystems.superstructure.Elevator;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
  // IO layer
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Mechanism2d visualization
  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(60, 60);
  private final LoggedMechanismRoot2d elevatorRoot = mech2d.getRoot("ElevatorBase", 30, 10);
  private final LoggedMechanismLigament2d elevatorLigament;
  private final LoggedMechanismLigament2d targetElevatorLigament;

  // Constants for visualization
  private static final double ELEVATOR_VISUAL_LENGTH_PER_TICK =
      0.05; // Adjust based on your tick scale
  private static final Distance MIN_ELEVATOR_VISUAL_LENGTH =
      Meters.of(0); // Minimum visualization length

  public Elevator(ElevatorIO io) {
    this.io = io;

    // Initialize elevator visualization - vertical orientation (90 degrees)
    elevatorLigament =
        elevatorRoot.append(
            new LoggedMechanismLigament2d(
                "Elevator", MIN_ELEVATOR_VISUAL_LENGTH, 90, 8, new Color8Bit(Color.kBlue)));

    targetElevatorLigament =
        elevatorRoot.append(
            new LoggedMechanismLigament2d(
                "TargetElevator", MIN_ELEVATOR_VISUAL_LENGTH, 90, 4, new Color8Bit(Color.kGreen)));

    // Publish mechanism to logger
    Logger.recordOutput("Mechanisms/Elevator", mech2d);
  }

  @Override
  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update elevator visualization
    Distance currentLength = MIN_ELEVATOR_VISUAL_LENGTH + inputs.currentHeight;

    elevatorLigament.setLength(currentLength.magnitude());

    Distance targetLength = MIN_ELEVATOR_VISUAL_LENGTH + inputs.targetHeight;

    targetElevatorLigament.setLength(targetLength.magnitude());
  }

  public void setElevatorHeight(Distance height) {
    io.setElevatorHeight(height);
  }

  public void setBrakes(IdleMode idleMode) {
    io.setBrakes(idleMode);
  }

  public void resetPosition(double position) {
    io.resetPosition(position);
  }

  public double getTicks() {
    return inputs.elevatorPosition;
  }

  public double getTargetPosition() {
    return inputs.targetHeight;
  }

  public ElevatorSensorState getSensorState() {
    return inputs.sensorState;
  }

  public void setPIDlimits(double lowerBound, double upperBound) {
    io.setPIDlimits(lowerBound, upperBound);
  }
}
