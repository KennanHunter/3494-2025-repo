package frc.robot.subsystems.SuperScructure;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator  extends SubsystemBase{
    private SparkMax leaderMotor;
    private SparkMaxConfig leaderConfig;

    private SparkMax followerMotor;
    private SparkMaxConfig followerConfig;

    private DigitalInput bottomMagSensor; 

    public double manualPower = 0;
    
    public  Elevator(){
        leaderMotor = new SparkMax(Constants.Elevator.leaderMotor, MotorType.kBrushless);
        followerMotor = new SparkMax(Constants.Elevator.followerMotor, MotorType.kBrushless);
        leaderConfig.idleMode(IdleMode.kCoast);
        followerConfig.follow(leaderMotor);

        leaderMotor.configure(
            leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(
            followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        bottomMagSensor = new DigitalInput(Constants.Elevator.bottomMagSensorDIO);
    }

    public void setElevatorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        leaderMotor.set(manualPower);
    }

    public void setElevatorVoltage(double voltage) {
        leaderMotor.getClosedLoopController().setReference(voltage, SparkBase.ControlType.kVoltage);
    }
    public void setBrakes(IdleMode newIdleMode){
        leaderConfig.idleMode(newIdleMode);
        leaderMotor.configure(leaderConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void periodic(){
        if (DriverStation.isEnabled()){
            this.setBrakes(IdleMode.kBrake);
        }

        if(getElevatorSensorState() == ElevatorSensorState.BOTTOM){
            leaderMotor.getEncoder().setPosition(0);
        }
    }
    /**
     * Combines the two Magnet Sensor inputs to generate an enum that can be used
     * for software limiting
     * 
     * @return {@link ElevatorSensorState} currentState
     */
    public ElevatorSensorState getElevatorSensorState() {
        if (!bottomMagSensor.get())
            return ElevatorSensorState.BOTTOM;
        return ElevatorSensorState.UP;
    }

    public void setElevatorPosition(double position) {
        leaderMotor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
    }

    public void resetPosition(double position) {
        leaderMotor.getEncoder().setPosition(position);
    }
    public double getManualMotorPower(){
        return manualPower;
    }
    public double getTicks(){
        return leaderMotor.getEncoder().getPosition();
    }

}
