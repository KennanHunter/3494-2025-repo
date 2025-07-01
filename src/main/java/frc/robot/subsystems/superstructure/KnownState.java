package frc.robot.subsystems.superstructure;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.Intake.IntakeState;

public enum KnownState {
  SafeReset(
      new SuperStructureState(
          new ElevatorState(Constants.Elevator.positionFromPercentageDecimal(0.7), IdleMode.kBrake),
          new ArmState(Rotation2d.kZero),
          IntakeState.Hold)),

  Test(
      new SuperStructureState(
          new ElevatorState(
              Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT, IdleMode.kBrake),
          new ArmState(Rotation2d.fromDegrees(45)),
          IntakeState.Hold)),

  Test2(
      new SuperStructureState(
          new ElevatorState(Constants.Elevator.positionFromPercentageDecimal(0.5), IdleMode.kBrake),
          new ArmState(Rotation2d.kCW_90deg),
          IntakeState.Spit));

  private SuperStructureState state;

  KnownState(SuperStructureState state) {
    this.state = state;
  }

  public SuperStructureState getState() {
    return this.state;
  }
}
