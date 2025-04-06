package frc.robot.subsystems.SuperStructure;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private SparkMax leaderMotor;
  private SparkMaxConfig leaderConfig;

  private SparkMax followerMotor;
  private SparkMaxConfig followerConfig;

  private DigitalInput bottomMagSensor;

  public double manualPower = 0;
  public double targetPosition = 99999.0;

  public Elevator() {
    leaderMotor = new SparkMax(Constants.Elevator.leaderMotor, MotorType.kBrushless);
    followerMotor = new SparkMax(Constants.Elevator.followerMotor, MotorType.kBrushless);
    leaderConfig = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();
    followerConfig.follow(leaderMotor, true);
    followerConfig.smartCurrentLimit(80);
    leaderConfig.smartCurrentLimit(80);

    leaderConfig.closedLoop.pid(0.5, 0, 0);
    leaderConfig.closedLoop.outputRange(-0.6, 0.6);
    leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    leaderConfig.idleMode(IdleMode.kCoast);
    leaderConfig.inverted(true);

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

  public void setBrakes(IdleMode newIdleMode) {
    leaderConfig.idleMode(newIdleMode);
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // if (DriverStation.isEnabled()) {
    //   this.setBrakes(IdleMode.kBrake);
    // }

    if (getElevatorSensorState() == ElevatorSensorState.BOTTOM) {
      leaderMotor.getEncoder().setPosition(0);
    }

    Logger.recordOutput("Elevator/Encoder-Position", leaderMotor.getEncoder().getPosition());
    Logger.recordOutput("Elevator/Sensor-Tripped", getElevatorSensorState());
    Logger.recordOutput("Elevator/Target-Position", targetPosition);
  }

  /**
   * Combines the two Magnet Sensor inputs to generate an enum that can be used for software
   * limiting
   *
   * @return {@link ElevatorSensorState} currentState
   */
  public ElevatorSensorState getElevatorSensorState() {
    if (!bottomMagSensor.get()) return ElevatorSensorState.BOTTOM;
    return ElevatorSensorState.UP;
  }

  public void setElevatorPosition(double position) {
    leaderMotor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
    targetPosition = position;
  }

  public void resetPosition(double position) {
    leaderMotor.getEncoder().setPosition(position);
  }

  public double getManualMotorPower() {
    return manualPower;
  }

  public double getTicks() {
    return leaderMotor.getEncoder().getPosition();
  }

  public void setPIDlimits(double lowerBound, double upperBound) {
    leaderConfig.closedLoop.outputRange(lowerBound, upperBound);
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
