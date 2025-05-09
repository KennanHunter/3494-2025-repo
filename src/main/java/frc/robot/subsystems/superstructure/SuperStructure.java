package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.Arm.Arm;
import frc.robot.subsystems.superstructure.Arm.ArmIO;
import frc.robot.subsystems.superstructure.Elevator.Elevator;
import frc.robot.subsystems.superstructure.Elevator.ElevatorIO;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  // Store the current rotation angle
  // private double currentAngle = 0.0;
  // Define rotation speed in radians per second
  // private final double ROTATION_SPEED = Math.PI / 4; // 45 degrees per second

  Elevator elevator;
  Arm arm;

  public SuperStructure(ElevatorIO elevatorIO, ArmIO armIO) {
    elevator = new Elevator(elevatorIO);
    arm = new Arm(armIO);
  }

  @Override
  public void periodic() {
    Optional<SuperStructureState> optionalSuperStructureState = getState();

    optionalSuperStructureState.ifPresent(
        (state) -> {
          Logger.recordOutput("SuperStructureState", state);
          Logger.recordOutput(
              "SuperStructureMechanismState", SuperStructureState.updateMechanism(state));
          Logger.recordOutput("SuperStructureComponents", SuperStructureState.updatePoses(state));
        });
  }

  Optional<SuperStructureState> getState() {
    return arm.getState().map((armState) -> new SuperStructureState(elevator.getState(), armState));
  }
}
