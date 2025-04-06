package frc.robot.subsystems.superstructure.Elevator;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax leaderMotor;
  private final SparkMax followerMotor;
  private SparkMaxConfig leaderConfig;
  private SparkMaxConfig followerConfig;
  private final DigitalInput bottomMagSensor;

  private double targetPosition = 0.0;
  private double manualPower = 0.0;

  public ElevatorIOSparkMax() {
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

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.targetHeight = targetPosition;

    // Update sensor state
    inputs.sensorState = getElevatorSensorState();

    // Reset encoder position if at bottom
    if (inputs.sensorState == ElevatorSensorState.BOTTOM) {
      leaderMotor.getEncoder().setPosition(0);
    }
  }

  @Override
  public void setElevatorHeight(Distance position) {

    // TODO: Properly map magnitude to reference
    leaderMotor
        .getClosedLoopController()
        .setReference(position.magnitude(), SparkBase.ControlType.kPosition);
    targetPosition = position.magnitude();
  }

  @Override
  public void setBrakes(IdleMode idleMode) {
    leaderConfig.idleMode(idleMode);
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void resetPosition(double position) {
    leaderMotor.getEncoder().setPosition(position);
  }

  @Override
  public void setPIDlimits(double lowerBound, double upperBound) {
    leaderConfig.closedLoop.outputRange(lowerBound, upperBound);
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private ElevatorSensorState getElevatorSensorState() {
    if (!bottomMagSensor.get()) return ElevatorSensorState.BOTTOM;
    return ElevatorSensorState.UP;
  }
}
