package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  // Store the current rotation angle
  private double currentAngle = 0.0;
  // Define rotation speed in radians per second
  private final double ROTATION_SPEED = Math.PI / 4; // 45 degrees per second

  @Override
  public void periodic() {
    // Update the current angle based on time
    currentAngle =
        (currentAngle + (ROTATION_SPEED * Constants.SIMULATED_LOOP_TIME)) % (2 * Math.PI);

    // Create a new ArmState with the updated rotation
    Rotation2d armRotation = new Rotation2d(currentAngle);

    SuperStructureState state =
        new SuperStructureState(
            new ElevatorState(Meters.of(1.2), MetersPerSecond.of(0), IdleMode.kBrake),
            new ArmState(armRotation));

    Logger.recordOutput("SuperStructureState", state);

    Logger.recordOutput("SuperStructureMechanismState", SuperStructureState.updateMechanism(state));
    Logger.recordOutput("SuperStructureComponents", SuperStructureState.updatePoses(state));
  }
}
