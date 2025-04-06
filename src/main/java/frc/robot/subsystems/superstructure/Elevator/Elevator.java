package frc.robot.subsystems.superstructure.Elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public class Elevator extends SubsystemBase {
  // IO layer
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Mechanism2d visualization
  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(60, 60);

  // private final LoggedMechanismRoot2d elevatorRoot = mech2d.getRoot("ElevatorBase", 30, 10);
  // private final LoggedMechanismLigament2d elevatorLigament;
  // private final LoggedMechanismLigament2d targetElevatorLigament;

  public Elevator(ElevatorIO io) {
    this.io = io;

    // Initialize elevator visualization - vertical orientation (90 degrees)
    // elevatorLigament =
    //     elevatorRoot.append(
    //         new LoggedMechanismLigament2d(
    //             Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT,
    //             90,
    //             8,
    //             new Color8Bit(Color.kBlue)));

    // targetElevatorLigament =
    //     elevatorRoot.append(
    //         new LoggedMechanismLigament2d(
    //             PHYSICAL_ELEVATOR_BOTTOM_HEIGHT, 90, 4, new Color8Bit(Color.kGreen)));

    // Publish mechanism to logger
    Logger.recordOutput("Mechanisms/Elevator", mech2d);
  }

  @Override
  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update elevator visualization
    // Distance currentLength = PHYSICAL_ELEVATOR_BOTTOM_HEIGHT + inputs.currentHeight;

    // elevatorLigament.setLength(currentLength.magnitude());

    // Distance targetLength = PHYSICAL_ELEVATOR_BOTTOM_HEIGHT + inputs.targetHeight;

    // targetElevatorLigament.setLength(targetLength.magnitude());
  }

  public void runHeight(Distance height) {}

  public ElevatorSensorState getSensorState() {
    return inputs.sensorState;
  }
}
