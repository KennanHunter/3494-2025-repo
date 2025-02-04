package frc.robot.subsystems.SuperScructure;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    SparkMax armMotor;
    SparkMaxConfig armMotorConfig;
    double manualPower = 0;
    private double targetPosition;
    public Arm(){
        armMotor = new SparkMax(Constants.Arm.armMotor, MotorType.kBrushless);
        armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(IdleMode.kCoast);
        // armMotor.getPIDController().setFF(0.5);
        // armMotor.getPIDController().setOutputRange(-0.7, 0.7);
        // armMotor.getPIDController().setFeedbackDevice(armMotor.getAlternateEncoder(8192));
        // armMotor.getPIDController().setFeedbackDevice(armMotor.getAbsoluteEncoder(Type.kDutyCycle));
        
    }
    public void setBrakes(IdleMode neutralMode) {
        this.armMotorConfig.idleMode(neutralMode);
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void setTargetAngle(double ticks, double arbFFVoltage) {
        targetPosition = ticks; 
        armMotor.getClosedLoopController().setReference(ticks, SparkMax.ControlType.kPosition);
    }
    @Override
    public void periodic(){
        if (DriverStation.isEnabled()) this.setBrakes(IdleMode.kBrake);
    }

    public void setMotorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        // System.out.println(manualPower);
        armMotor.set(manualPower);
    }
    public double getManualMotorPower(){
        return manualPower;
    }
    public double getRelativeTicks(){
        return armMotor.getEncoder().getPosition();
    }
    public double getAbsoluteTicks(){
        return armMotor.getEncoder().getPosition();
        // return armMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }
    public double getTargetPosition(){
        return targetPosition;
    }
}
