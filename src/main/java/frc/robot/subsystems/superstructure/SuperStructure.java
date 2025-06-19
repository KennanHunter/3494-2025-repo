package frc.robot.subsystems.superstructure;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.Arm.Arm;
import frc.robot.subsystems.superstructure.Arm.ArmIO;
import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.Elevator;
import frc.robot.subsystems.superstructure.Elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.Intake.Intake;
import frc.robot.subsystems.superstructure.Intake.IntakeState;
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

  public SuperStructure(ElevatorIO elevatorIO, ArmIO armIO) {
    elevator = new Elevator(elevatorIO);
    arm = new Arm(armIO);
    intake = new Intake();

    currentPositionVisualizer =
        new SuperStructureVisualizer(new Color8Bit(Color.kBlue), new Color8Bit(Color.kAqua));
    targetPositionVisualizer =
        new SuperStructureVisualizer(new Color8Bit(Color.kGreen), new Color8Bit(Color.kLightGreen));

    setTargetState(
        new SuperStructureState(
            new ElevatorState(Constants.Elevator.positionFromPercentage(0.5), IdleMode.kCoast),
            new ArmState(Rotation2d.kZero),
            IntakeState.Hold));
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
  }

  Optional<SuperStructureState> getState() {
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
}
