package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;

public record SuperStructureState(ElevatorState elevatorState, ArmState armState) {}
