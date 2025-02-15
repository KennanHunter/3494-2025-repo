package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  SparkMax climberMotor;
  SparkMaxConfig climberMotorConfig;
  
  double manualPower = 0;
  private double targetPosition;
  private RelativeEncoder encoder;

  public Climber() {
    climberMotor = new SparkMax(Constants.Climber.climberMotor, MotorType.kBrushless);
    climberMotorConfig = new SparkMaxConfig();
    climberMotorConfig.idleMode(IdleMode.kCoast);
    climberMotorConfig.inverted(false);
    climberMotorConfig.closedLoop.pid(2, 0, 0);
    climberMotorConfig.closedLoop.outputRange(-1, 1);
    climberMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    climberMotorConfig.smartCurrentLimit(80);
    climberMotor.configure(
        climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTargetAngle(double ticks, double arbFFVoltage) {
    targetPosition = ticks;
    climberMotor.getClosedLoopController().setReference(ticks, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Clibmer/Climber-Position", climberMotor.getEncoder().getPosition());
    Logger.recordOutput("Climber/Target-Position", targetPosition);
    Logger.recordOutput("Climber/Manual-Power", manualPower);
    Logger.recordOutput("Climber/Applied-Power", climberMotor.getAppliedOutput());
    

  }

  public void setMotorPower(double power) {
    power = Math.max(Math.min(power, 1), -1);
    manualPower = power;
    // System.out.println(manualPower);
    if(climberMotor.getEncoder().getPosition() > -1){
      climberMotor.set(manualPower);
    }
    
  }

  public double getManualMotorPower() {
    return manualPower;
  }

  public double getRelativeTicks() {
    return climberMotor.getEncoder().getPosition();
  }
  public double getTargetPosition() {
    return targetPosition;
  }
}
