package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  @Override
  public void periodic() {
    SuperStructureState state =
        new SuperStructureState(
            new ElevatorState(Meters.of(1.2), MetersPerSecond.of(0), IdleMode.kBrake),
            new ArmState(Rotation2d.kCW_90deg));

    Logger.recordOutput("SuperStructureState", SuperStructureState.updateMechanism(state));
  }
}
