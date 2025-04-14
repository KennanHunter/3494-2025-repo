package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.Arm.Arm;
import frc.robot.subsystems.superstructure.Arm.ArmIOSim;
import frc.robot.subsystems.superstructure.Elevator.Elevator;
import frc.robot.subsystems.superstructure.Elevator.ElevatorIOSim;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  // Store the current rotation angle
  private double currentAngle = 0.0;
  // Define rotation speed in radians per second
  private final double ROTATION_SPEED = Math.PI / 4; // 45 degrees per second

  Elevator elevator;
  Arm arm;

  public SuperStructure() {
    elevator = new Elevator(new ElevatorIOSim());

    arm = new Arm(new ArmIOSim());
  }

  @Override
  public void periodic() {
    SuperStructureState state = getState();

    Logger.recordOutput("SuperStructureState", state);

    Logger.recordOutput("SuperStructureMechanismState", SuperStructureState.updateMechanism(state));
    Logger.recordOutput("SuperStructureComponents", SuperStructureState.updatePoses(state));
  }

  SuperStructureState getState() {
    return new SuperStructureState(elevator.getState(), arm.getState());
  }
}
