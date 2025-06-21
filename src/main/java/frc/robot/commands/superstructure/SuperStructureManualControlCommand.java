package frc.robot.commands.superstructure;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructureState;
import java.util.function.Supplier;

// Create a default command that handles both manual and automatic control
public class SuperStructureManualControlCommand extends Command {
  private final SuperStructure superStructure;
  private final Supplier<Double> elevatorInput;
  private final Supplier<Double> armInput;

  public SuperStructureManualControlCommand(
      SuperStructure superStructure, Supplier<Double> elevatorInput, Supplier<Double> armInput) {
    this.superStructure = superStructure;
    this.elevatorInput = elevatorInput;
    this.armInput = armInput;

    addRequirements(superStructure);
  }

  @Override
  public void execute() {
    handleManualControl();
  }

  private void handleManualControl() {
    double elevatorAdjustment = elevatorInput.get() * 0.02; // 2cm per command cycle
    double armAdjustment = armInput.get() * Math.toRadians(2); // 2 degrees per cycle

    var currentState = superStructure.getState();
    if (currentState.isPresent()) {
      var state = currentState.get();

      var newElevatorPos = state.elevatorState().height().plus(Meters.of(elevatorAdjustment));
      var newArmAngle = state.armState().rotation().plus(Rotation2d.fromRadians(armAdjustment));

      superStructure.setTargetState(
          new SuperStructureState(
              new ElevatorState(newElevatorPos, IdleMode.kBrake),
              new ArmState(newArmAngle),
              state.intakeState()));
    }
  }
}
