package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.KnownState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructureMachine;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.jgrapht.GraphPath;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.Logger;

/** The real entrypoint for this command is SuperStructure::traverseToKnownState */
public class SuperStructureTraverseCommand extends Command {
  static Alert detachedStateAlert =
      new Alert("Tried to traverse while outside of state machine", AlertType.kWarning);

  private final SuperStructure superStructure;
  private final KnownState targetState;
  private final Supplier<Optional<KnownState>> currentStateSupplier;

  private GraphPath<KnownState, DefaultEdge> path;
  private List<KnownState> stateSequence;
  private int currentStepIndex;
  private boolean pathCalculated = false;

  /**
   * Creates a command to traverse from current state to target state
   *
   * @param superStructure The superstructure subsystem
   * @param currentStateSupplier Supplier that provides the current known state
   * @param targetState The target known state to reach
   */
  public SuperStructureTraverseCommand(
      SuperStructure superStructure,
      Supplier<Optional<KnownState>> currentStateSupplier,
      KnownState targetState) {
    this.superStructure = superStructure;
    this.targetState = targetState;
    this.currentStateSupplier = currentStateSupplier;

    addRequirements(superStructure);
  }

  /**
   * Creates a command to traverse from a specific start state to target state
   *
   * @param superStructure The superstructure subsystem
   * @param startState The starting known state
   * @param targetState The target known state to reach
   */
  public SuperStructureTraverseCommand(
      SuperStructure superStructure, KnownState startState, KnownState targetState) {
    this(superStructure, () -> Optional.of(startState), targetState);
  }

  @Override
  public void initialize() {
    if (currentStateSupplier.get().isEmpty()) {
      detachedStateAlert.set(true);
      return;
    }
    detachedStateAlert.set(false);

    KnownState currentState = currentStateSupplier.get().get();

    path = SuperStructureMachine.traverse(currentState, targetState);

    if (path == null || path.getVertexList().isEmpty()) {
      Logger.recordOutput("SuperStructureTraverse/PathFound", false);
      // TODO: Turn this into alert
      System.err.print("No path found from " + currentState + " to " + targetState);

      pathCalculated = false;
      return;
    }

    stateSequence = path.getVertexList();
    currentStepIndex = 0;
    pathCalculated = true;

    Logger.recordOutput("SuperStructureTraverse/PathFound", true);
    Logger.recordOutput("SuperStructureTraverse/PathLength", stateSequence.size());
    Logger.recordOutput("SuperStructureTraverse/StartState", currentState.toString());
    Logger.recordOutput("SuperStructureTraverse/TargetState", targetState.toString());
    Logger.recordOutput(
        "SuperStructureTraverse/FullPath",
        stateSequence.stream().map(KnownState::toString).toArray(String[]::new));

    if (!stateSequence.isEmpty()) {
      executeCurrentStep();
    }
  }

  @Override
  public void execute() {
    if (currentStateSupplier.get().isEmpty()) {
      detachedStateAlert.set(true);
      return;
    }
    detachedStateAlert.set(false);

    if (!pathCalculated || stateSequence.isEmpty()) {
      return;
    }

    Logger.recordOutput("SuperStructureTraverse/CurrentStep", currentStepIndex);
    Logger.recordOutput(
        "SuperStructureTraverse/CurrentTargetState", getCurrentTargetState().toString());
    Logger.recordOutput("SuperStructureTraverse/IsAtTarget", superStructure.isAtTarget());
    Logger.recordOutput("SuperStructureTraverse/Interrupted", false);

    if (superStructure.isAtTarget() && getRemainingSteps() != 0) {
      currentStepIndex++;
      executeCurrentStep();
    }
  }

  @Override
  public boolean isFinished() {
    if (!pathCalculated) {
      return true; // Finish immediately if no path was found
    }

    // Command is finished when we've reached the final target state
    return currentStepIndex >= stateSequence.size() - 1 && superStructure.isAtTarget();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Logger.recordOutput("SuperStructureTraverse/Interrupted", true);
      Logger.recordOutput("SuperStructureTraverse/CompletedSteps", currentStepIndex);
    } else {
      Logger.recordOutput("SuperStructureTraverse/Completed", true);
    }

    Logger.recordOutput("SuperStructureTraverse/Active", false);
  }

  /** Executes the current step in the state sequence */
  private void executeCurrentStep() {
    if (currentStepIndex < stateSequence.size()) {
      KnownState currentTargetState = stateSequence.get(currentStepIndex);
      superStructure.setTargetKnownState(currentTargetState);

      Logger.recordOutput("SuperStructureTraverse/ExecutingStep", currentStepIndex);
      Logger.recordOutput("SuperStructureTraverse/StepState", currentTargetState.toString());
    } else {
      System.err.println(
          "Tried to execute step "
              + currentStepIndex
              + " that doesn't exist in stateSequence with length "
              + stateSequence.size());
    }
  }

  /** Gets the current target state in the sequence */
  private KnownState getCurrentTargetState() {
    if (currentStepIndex < stateSequence.size()) {
      return stateSequence.get(currentStepIndex);
    }
    return targetState;
  }

  /** Gets the remaining steps in the current path */
  public int getRemainingSteps() {
    if (!pathCalculated || stateSequence.isEmpty()) {
      return 0;
    }
    return Math.max(0, stateSequence.size() - currentStepIndex - 1);
  }

  /** Gets the total number of steps in the path */
  public int getTotalSteps() {
    return pathCalculated ? stateSequence.size() : 0;
  }
}
