package frc.robot.subsystems.superstructure.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {
  // SparkMax hardware objects
  private final SparkMax leaderMotor;
  // private final SparkMax followerMotor;
  private final SparkMaxConfig leaderConfig;
  // private final SparkMaxConfig followerConfig;

  // Limit switch for bottom position detection
  private final DigitalInput bottomLimitSwitch;

  public ElevatorIOReal() {
    // Create SparkMax motors with the IDs from constants
    leaderMotor = new SparkMax(Constants.Elevator.leaderMotor, MotorType.kBrushless);
    // followerMotor = new SparkMax(Constants.Elevator.followerMotor, MotorType.kBrushless);

    // Create configs
    leaderConfig = new SparkMaxConfig();
    // followerConfig = new SparkMaxConfig();

    // Configure follower to mirror leader with inverted output
    // followerConfig.follow(leaderMotor, true);

    // Set current limits to prevent motor damage
    // followerConfig.smartCurrentLimit(80);
    leaderConfig.smartCurrentLimit(80);

    // Configure PID and motion control
    leaderConfig.closedLoop.pid(0.5, 0, 0);
    leaderConfig.closedLoop.outputRange(-0.6, 0.6);
    leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    leaderConfig.closedLoop.maxOutput(0);

    // TODO: WHY THE ACTUAL FLYING FUCK DOES THIS CONVERSION FACTOR ERROR

    // leaderConfig.encoder.positionConversionFactor(
    //     Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO);

    // leaderConfig.encoder.velocityConversionFactor(
    //     Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO / 60);

    leaderConfig.idleMode(IdleMode.kCoast);

    // Motor direction
    leaderConfig.inverted(true);

    // Apply configurations
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // followerMotor.configure(
    //     followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize the bottom limit switch on the specified DIO port
    bottomLimitSwitch = new DigitalInput(Constants.Elevator.bottomMagSensorDIO);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Read current elevator position from encoder
    inputs.currentHeight =
        Inches.of(
            leaderMotor.getEncoder().getPosition()
                * Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO);

    boolean isBottomSwitchPressed = !bottomLimitSwitch.get();
    inputs.sensorState =
        isBottomSwitchPressed ? ElevatorSensorState.BOTTOM : ElevatorSensorState.UP;

    Logger.recordOutput(
        "Elevator/Leader/RotToInchesConvRatio",
        Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO);
    Logger.recordOutput("Elevator/Leader/Rotations", leaderMotor.getEncoder().getPosition());
    Logger.recordOutput("Elevator/Leader/AppliedAmps", leaderMotor.getOutputCurrent());
    Logger.recordOutput("Elevator/BottomSwitchPressed", isBottomSwitchPressed);
  }

  @Override
  public void runElevatorHeight(Distance height) {
    // Calculate the target position, accounting for physical offset
    Distance computedElevatorHeight =
        height.minus(Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT);

    double targetPosition = computedElevatorHeight.in(Meters);

    // Log the target position for debugging
    Logger.recordOutput("Elevator/TargetPositionMeters", targetPosition);

    // Command the motor to the target position using closed-loop control
    leaderMotor
        .getClosedLoopController()
        .setReference(targetPosition, SparkBase.ControlType.kPosition);
  }

  @Override
  public void setBrakes(IdleMode idleMode) {
    // Update the idle mode configuration
    leaderConfig.idleMode(idleMode);
    // followerConfig.idleMode(idleMode);

    // Apply the configuration to both motors
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // followerMotor.configure(
    // followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Logger.recordOutput("Elevator/BrakeMode", idleMode.toString());
  }

  @Override
  public void resetPosition(double position) {
    // Reset the encoder position
    leaderMotor.getEncoder().setPosition(position);
    Logger.recordOutput("Elevator/ResetPosition", position);
  }

  @Override
  public void setPIDlimits(double lowerBound, double upperBound) {
    // Update the PID output limits
    leaderConfig.closedLoop.outputRange(lowerBound, upperBound);

    // Apply the updated configuration
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Logger.recordOutput("Elevator/PIDLowerLimit", lowerBound);
    Logger.recordOutput("Elevator/PIDUpperLimit", upperBound);
  }
}
