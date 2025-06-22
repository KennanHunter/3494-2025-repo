package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.superstructure.SuperStructureTraverseCommand;
import frc.robot.subsystems.superstructure.Arm.Arm;
import frc.robot.subsystems.superstructure.Arm.ArmIO;
import frc.robot.subsystems.superstructure.Elevator.Elevator;
import frc.robot.subsystems.superstructure.Elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.Intake.Intake;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  // Store the current rotation angle
  // private double currentAngle = 0.0;
  // Define rotation speed in radians per second
  // private final double ROTATION_SPEED = Math.PI / 4; // 45 degrees per second

  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  private final SuperStructureVisualizer currentPositionVisualizer;
  private final SuperStructureVisualizer targetPositionVisualizer;

  private KnownState lastKnownState;

  public SuperStructure(ElevatorIO elevatorIO, ArmIO armIO) {
    elevator = new Elevator(elevatorIO);
    arm = new Arm(armIO);
    intake = new Intake();

    currentPositionVisualizer =
        new SuperStructureVisualizer(new Color8Bit(Color.kBlue), new Color8Bit(Color.kAqua));
    targetPositionVisualizer =
        new SuperStructureVisualizer(new Color8Bit(Color.kGreen), new Color8Bit(Color.kLightGreen));
  }

  @Override
  public void periodic() {
    getState()
        .ifPresent(
            (state) -> {
              Logger.recordOutput("SuperStructureState", state);
              Logger.recordOutput(
                  "SuperStructureMechanismState", currentPositionVisualizer.updateMechanism(state));
              Logger.recordOutput(
                  "SuperStructureComponents", currentPositionVisualizer.updatePoses(state));
            });

    Logger.recordOutput("SuperStructure/isAtTargetState", isAtTarget());
    Logger.recordOutput("SuperStructure/lastKnownState", lastKnownState);
    Logger.recordOutput(
        "SuperStructure/isWithinRangeOfLastKnownState", isWithinRangeOfKnownState(lastKnownState));
  }

  public Optional<SuperStructureState> getState() {
    if (arm.getState().isEmpty() || intake.getState().isEmpty()) return Optional.empty();

    return Optional.of(
        new SuperStructureState(
            elevator.getState(), arm.getState().get(), intake.getState().get()));
  }

  public void setTargetState(SuperStructureState newState) {
    elevator.setTargetState(newState.elevatorState());
    arm.setTargetState(newState.armState());
    intake.setTargetState(newState.intakeState());

    Logger.recordOutput(
        "TargetSuperStructureMechanismState", targetPositionVisualizer.updateMechanism(newState));
    Logger.recordOutput(
        "TargetSuperStructureComponents", targetPositionVisualizer.updatePoses(newState));
  }

  public boolean isAtTarget() {
    return elevator.isAtTarget() && arm.isAtTarget() && intake.isAtTarget();
  }

  public void setTargetKnownState(KnownState state) {
    this.lastKnownState = state;

    setTargetState(state.getState());
  }

  // TODO: Merge with isAtTarget()
  public boolean isWithinRangeOfKnownState(KnownState state) {
    if (this.getState().isEmpty()) {
      return false;
    }

    var old = this.getState().get();
    var newState = state.getState();

    boolean armWithinRange =
        Math.abs(old.armState().rotation().minus(newState.armState().rotation()).getDegrees())
            <= arm.ACCEPTABLE_ANGLE_ERROR.in(Degrees);

    boolean heightWithinRange =
        old.elevatorState().height().minus(newState.elevatorState().height()).abs(Centimeters)
            <= elevator.ELEVATOR_ACCEPTABLE_HEIGHT_ERROR.in(Centimeters);

    return armWithinRange && heightWithinRange;
  }

  public Optional<KnownState> getLastKnownStateIfWithinRange() {
    if (this.lastKnownState == null) {
      return Optional.empty();
    }

    if (this.isWithinRangeOfKnownState(this.lastKnownState)) {
      return Optional.of(this.lastKnownState);
    } else {
      return Optional.empty();
    }
  }

  public Command createCommandTraversalToKnownState(KnownState targetState) {
    return new SuperStructureTraverseCommand(
            this, () -> getLastKnownStateIfWithinRange(), targetState)
        .onlyIf(() -> getLastKnownStateIfWithinRange().isPresent());
  }
}
