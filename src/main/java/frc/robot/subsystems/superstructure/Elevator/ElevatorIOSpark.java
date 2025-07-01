package frc.robot.subsystems.superstructure.Elevator;

import static edu.wpi.first.units.Units.Inches;

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

public class ElevatorIOSpark implements ElevatorIO {
  // SparkMax hardware objects
  private final SparkMax leaderMotor;
  private final SparkMax followerMotor;

  private final SparkMaxConfig leaderConfig;

  // Limit switch for bottom position detection
  private final DigitalInput bottomLimitSwitch;

  public ElevatorIOSpark() {
    // Create SparkMax motors with the IDs from constants
    leaderMotor = new SparkMax(Constants.Elevator.leaderMotor, MotorType.kBrushless);
    followerMotor = new SparkMax(Constants.Elevator.followerMotor, MotorType.kBrushless);

    // TODO: Actually hook follower motor in
    followerMotor.configure(
        new SparkMaxConfig().idleMode(IdleMode.kCoast),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Create configs
    leaderConfig = new SparkMaxConfig();
    leaderConfig.smartCurrentLimit(Constants.Arm.ARM_STALL_CURRENT_LIMIT);

    // Configure PID and motion control
    leaderConfig.closedLoop.pid(1, 0, 0);
    leaderConfig.closedLoop.outputRange(-0.2, 0.2);
    leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    leaderConfig.idleMode(IdleMode.kCoast);

    // Motor direction
    leaderConfig.inverted(true);

    // Apply configurations
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    bottomLimitSwitch = new DigitalInput(Constants.Elevator.bottomMagSensorDIO);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Read current elevator position from encoder
    inputs.currentHeight =
        Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT.plus(
            Inches.of(
                leaderMotor.getEncoder().getPosition()
                    * Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO));

    boolean isBottomSwitchPressed = !bottomLimitSwitch.get();
    inputs.sensorState =
        isBottomSwitchPressed ? ElevatorSensorState.BOTTOM : ElevatorSensorState.UP;

    Logger.recordOutput(
        "Elevator/Leader/RotToInchesConversionRatio",
        Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO);
    Logger.recordOutput("Elevator/Leader/Rotations", leaderMotor.getEncoder().getPosition());
    Logger.recordOutput("Elevator/Leader/AppliedAmps", leaderMotor.getOutputCurrent());
    Logger.recordOutput("Elevator/BottomSwitchPressed", isBottomSwitchPressed);
  }

  @Override
  public void runElevatorHeight(Distance height) {
    // Calculate the target position, accounting for physical offset
    Distance computedElevatorHeight =
        height.minus(Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT);

    double targetPosition = computedElevatorHeight.in(Inches);

    // Log the target position for debugging
    Logger.recordOutput("Elevator/TargetPositionInches", targetPosition);

    // Command the motor to the target position using closed-loop control
    leaderMotor
        .getClosedLoopController()
        .setReference(
            targetPosition / Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO,
            SparkBase.ControlType.kPosition);
  }

  @Override
  public void setBrakes(IdleMode idleMode) {
    leaderConfig.idleMode(idleMode);

    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Logger.recordOutput("Elevator/BrakeMode", idleMode.toString());
  }

  @Override
  public void resetHeight(Distance height) {
    // Reset the encoder position
    leaderMotor
        .getEncoder()
        .setPosition(
            height.minus(Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT).in(Inches)
                / Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO);
    Logger.recordOutput("Elevator/ResetHeight", height);
  }
}
