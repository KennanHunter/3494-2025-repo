package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.Intake.IntakeState;

public enum KnownStates {
  Test(
      new SuperStructureState(
          new ElevatorState(Meters.of(0.5), IdleMode.kBrake),
          new ArmState(Rotation2d.kZero),
          IntakeState.Hold)),

  Test2(
      new SuperStructureState(
          new ElevatorState(Meters.of(0.7), IdleMode.kBrake),
          new ArmState(Rotation2d.kCW_90deg),
          IntakeState.Spit));

  KnownStates(SuperStructureState state) {}
}
