package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.Intake.IntakeState;

public record SuperStructureState(
    ElevatorState elevatorState, ArmState armState, IntakeState intakeState) {}
