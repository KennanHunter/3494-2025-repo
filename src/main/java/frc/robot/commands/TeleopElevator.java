package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.SuperStructure.Elevator;

public class TeleopElevator extends Command {
    Elevator elevator;
    private double elevatorPower = 0;
    public TeleopElevator(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void execute() {
        elevatorPower = OI.deadband(OI.getElevatorPower(), 0.05);
        Logger.recordOutput("Elevator/Manual-Power", elevatorPower);
        if(elevatorPower != 0 || (elevator.getManualMotorPower() != 0 && elevatorPower == 0)){
            elevator.setElevatorPower(elevatorPower);
        }

    }
    
}
