package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SuperStructureTraverseCommand;
import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.Intake.IntakeState;
import frc.robot.subsystems.superstructure.KnownState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructureState;

public final class OI {
  private static EventLoop eventLoop = new EventLoop();
  public static XboxController primaryController =
      new XboxController(Constants.OI.PRIMARY_CONTROLLER_PORT);
  public static Joystick rightButtonBoard = new Joystick(2);
  public static Joystick leftButtonBoard = new Joystick(1);

  // SuperStructure control state
  private static SuperStructure superStructure;
  private static boolean manualMode = false;
  private static final CommandScheduler scheduler = CommandScheduler.getInstance();

  /**
   * Initialize the OI with subsystem references for command binding Call this from RobotContainer
   * after creating subsystems
   */
  public static void initialize(SuperStructure superStructureSubsystem) {
    superStructure = superStructureSubsystem;
    configureCommandBindings();
  }

  /** Configure command bindings for SuperStructure automation and manual control */
  private static void configureCommandBindings() {
    if (superStructure == null) return;

    // Manual mode toggle - Left and Right bumpers
    primaryController
        .leftBumper(eventLoop)
        .rising()
        .ifHigh(
            Commands.runOnce(
                () -> {
                  scheduler.cancel(superStructure); // Cancel any running commands
                  manualMode = true;
                  // Set current state as baseline for manual adjustments
                  var currentState = superStructure.getState();
                  if (currentState.isPresent()) {
                    superStructure.setTargetState(currentState.get());
                  }
                }));

    primaryController.rightBumper(eventLoop).onTrue(Commands.runOnce(() -> manualMode = false));

    // SuperStructure state commands (only run when not in manual mode)
    // Using Xbox controller buttons for quick access
    primaryController
        .a(eventLoop)
        .onTrue(
            new SuperStructureTraverseCommand(
                    superStructure, OI::getCurrentKnownState, KnownState.Test)
                .unless(() -> manualMode));

    primaryController
        .b(eventLoop)
        .onTrue(
            new SuperStructureTraverseCommand(
                    superStructure, OI::getCurrentKnownState, KnownState.Test2)
                .unless(() -> manualMode));

    // Add more state transitions as needed
    // primaryController.x(eventLoop).onTrue(
    //     new SuperStructureTraverseCommand(
    //         superStructure,
    //         OI::getCurrentKnownState,
    //         KnownState.YourNewState
    //     ).unless(() -> manualMode)
    // );
  }

  public static XboxController getPrimaryController() {
    return primaryController;
  }

  public static void update() {
    eventLoop.poll();

    // Handle manual superstructure adjustments when in manual mode
    if (manualMode && superStructure != null) {
      handleManualSuperStructureControl();
    }
  }

  /** Handle manual superstructure control when in manual mode */
  private static void handleManualSuperStructureControl() {
    var currentState = superStructure.getState();
    if (currentState.isEmpty()) return;

    var state = currentState.get();
    boolean stateChanged = false;

    // Manual elevator control via POV (same as your existing getElevatorPower logic)
    if (primaryController.povUp(eventLoop).getAsBoolean()) {
      var newElevatorState =
          new ElevatorState(
              state.elevatorState().position().plus(Meters.of(0.01)), // 1cm increment
              IdleMode.kBrake);
      state = new SuperStructureState(newElevatorState, state.armState(), state.intakeState());
      stateChanged = true;
    } else if (primaryController.povDown(eventLoop).getAsBoolean()) {
      var newElevatorState =
          new ElevatorState(
              state.elevatorState().position().minus(Meters.of(0.01)), // 1cm decrement
              IdleMode.kBrake);
      state = new SuperStructureState(newElevatorState, state.armState(), state.intakeState());
      stateChanged = true;
    }

    // Manual arm control via POV (same as your existing getArmPower logic)
    if (primaryController.povLeft(eventLoop).getAsBoolean()) {
      var newArmState =
          new ArmState(
              state.armState().angle().plus(Rotation2d.fromDegrees(1)) // 1 degree increment
              );
      state = new SuperStructureState(state.elevatorState(), newArmState, state.intakeState());
      stateChanged = true;
    } else if (primaryController.povRight(eventLoop).getAsBoolean()) {
      var newArmState =
          new ArmState(
              state.armState().angle().minus(Rotation2d.fromDegrees(1)) // 1 degree decrement
              );
      state = new SuperStructureState(state.elevatorState(), newArmState, state.intakeState());
      stateChanged = true;
    }

    // Manual intake control via triggers
    if (primaryController.leftTrigger(eventLoop).getAsBoolean()) {
      state = new SuperStructureState(state.elevatorState(), state.armState(), IntakeState.Spit);
      stateChanged = true;
    } else if (primaryController.rightTrigger(eventLoop).getAsBoolean()) {
      state = new SuperStructureState(state.elevatorState(), state.armState(), IntakeState.Hold);
      stateChanged = true;
    }

    // Apply changes if any were made
    if (stateChanged) {
      superStructure.setTargetState(state);
    }
  }

  /**
   * Determine current known state based on superstructure position This is a simplified
   * implementation - you may want to add tolerance checking
   */
  private static KnownState getCurrentKnownState() {
    if (superStructure == null) return KnownState.Test; // Default fallback

    var currentState = superStructure.getState();
    if (currentState.isEmpty()) return KnownState.Test;

    var state = currentState.get();

    // Check if current state matches Test (0.5m elevator, 0° arm, Hold intake)
    if (isStateClose(
        state,
        new ElevatorState(Meters.of(0.5), IdleMode.kBrake),
        new ArmState(Rotation2d.kZero),
        IntakeState.Hold)) {
      return KnownState.Test;
    }

    // Check if current state matches Test2 (0.7m elevator, 90° arm, Spit intake)
    if (isStateClose(
        state,
        new ElevatorState(Meters.of(0.7), IdleMode.kBrake),
        new ArmState(Rotation2d.kCW_90deg),
        IntakeState.Spit)) {
      return KnownState.Test2;
    }

    // Default to Test if no close match found
    return KnownState.Test;
  }

  /** Check if current state is close to a target state (with tolerance) */
  private static boolean isStateClose(
      SuperStructureState current,
      ElevatorState targetElevator,
      ArmState targetArm,
      IntakeState targetIntake) {
    final double ELEVATOR_TOLERANCE = 0.05; // 5cm tolerance
    final double ARM_TOLERANCE = Math.toRadians(5); // 5 degree tolerance

    boolean elevatorClose =
        Math.abs(
                current.elevatorState().position().in(Meters)
                    - targetElevator.position().in(Meters))
            < ELEVATOR_TOLERANCE;

    boolean armClose =
        Math.abs(current.armState().angle().getRadians() - targetArm.angle().getRadians())
            < ARM_TOLERANCE;

    boolean intakeMatch = current.intakeState() == targetIntake;

    return elevatorClose && armClose && intakeMatch;
  }

  /** Get whether we're currently in manual control mode */
  public static boolean isManualMode() {
    return manualMode;
  }

  // ===== EXISTING METHODS (unchanged) =====

  public static double deadband(double input, double deadband) {
    if (Math.abs(input) > deadband) {
      if (input > 0.0) {
        return (input - deadband) / (1.0 - deadband);
      }

      return (input + deadband) / (1.0 - deadband);
    }

    return 0.0;
  }

  public static double getIntakePower() {
    // TODO: Properly compute intake power as wanted
    return 0;
  }

  public static double getElevatorPower() {
    // NOTE: This method is preserved for compatibility, but manual mode
    // now handles elevator control through the SuperStructure
    if (manualMode) {
      return 0.0; // SuperStructure handles control in manual mode
    }

    // TODO: assign a button
    double upPower = (primaryController.povUp(eventLoop).getAsBoolean() ? -0.5 : 0.0);
    double downPower = (primaryController.povDown(eventLoop).getAsBoolean() ? 0.5 : 0.0);
    return upPower + downPower;
  }

  public static double getArmPower() {
    // NOTE: This method is preserved for compatibility, but manual mode
    // now handles arm control through the SuperStructure
    if (manualMode) {
      return 0.0; // SuperStructure handles control in manual mode
    }

    double leftPower = (primaryController.povLeft(eventLoop).getAsBoolean() ? 1 : 0.0);
    double rightPower = (primaryController.povRight(eventLoop).getAsBoolean() ? -1 : 0.0);
    return leftPower + rightPower;
  }

  public static double getClimberPower() {
    double upPower = (primaryController.povUp(eventLoop).getAsBoolean() ? 0.5 : 0.0);
    double downPower = (primaryController.povDown(eventLoop).getAsBoolean() ? -0.25 : 0.0);
    return upPower + downPower;
  }

  public static BooleanEvent bargeYeet() {
    return leftButtonBoard.button(7, eventLoop);
  }

  public static BooleanEvent lowIntake() {
    return leftButtonBoard.button(10, eventLoop);
  }

  public static BooleanEvent lowLowIntake() {
    return rightButtonBoard.button(2, eventLoop);
  }

  public static BooleanEvent lolipop() {
    return rightButtonBoard.button(1, eventLoop);
  }

  public static BooleanEvent Intake() {
    return leftButtonBoard.button(6, eventLoop);
  }

  public static BooleanEvent Processor() {
    return leftButtonBoard.button(8, eventLoop);
  }

  public static BooleanEvent L3Algea() {
    return leftButtonBoard.button(1, eventLoop);
  }

  public static BooleanEvent L3Coral() {
    return leftButtonBoard.button(2, eventLoop);
  }

  public static BooleanEvent startClimb() {
    return rightButtonBoard.button(3, eventLoop);
  }

  public static BooleanEvent ClimbStage0() {
    return rightButtonBoard.button(5, eventLoop);
  }

  public static BooleanEvent ClimbStage1() {
    return rightButtonBoard.button(6, eventLoop);
  }

  public static BooleanEvent ClimbStage2() {
    return rightButtonBoard.button(7, eventLoop);
  }
}
