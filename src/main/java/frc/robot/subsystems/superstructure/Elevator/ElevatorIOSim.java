package frc.robot.subsystems.superstructure.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

// TODO: The simulation part of this is really scuffed and likely fixed by just getting all the
// actual values we need for ElevatorSim
public class ElevatorIOSim implements ElevatorIO {
  // SparkMax hardware objects
  private final SparkMax leaderMotor;
  private final SparkMaxConfig leaderConfig;

  // Simulation components
  private final DCMotor elevatorMotorModel = DCMotor.getNEO(1);
  private final SparkMaxSim motorSim;
  private final ElevatorSim elevatorSim;

  public ElevatorIOSim() {
    // Create SparkMax motors with the IDs from constants
    leaderMotor = new SparkMax(Constants.Elevator.leaderMotor, MotorType.kBrushless);

    // Create configs
    leaderConfig = new SparkMaxConfig();
    leaderConfig.smartCurrentLimit(Constants.Arm.ARM_STALL_CURRENT_LIMIT);

    // Configure PID and motion control
    leaderConfig.closedLoop.pid(1, 0, 0);
    leaderConfig.closedLoop.outputRange(-1, 1);
    leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    leaderConfig.idleMode(IdleMode.kCoast);

    // Motor direction
    leaderConfig.inverted(true);

    // Apply configurations
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize simulation
    motorSim = new SparkMaxSim(leaderMotor, elevatorMotorModel);
    elevatorSim =
        new ElevatorSim(
            elevatorMotorModel,
            1,
            0.1,
            DRUM_RADIUS,
            Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT.in(Meters),
            Constants.Elevator.PHYSICAL_ELEVATOR_TOP_HEIGHT_MEASUREMENT.in(Meters),
            true,
            Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT.in(Meters));
  }

  static double DRUM_RADIUS = 0.00178;

  private void stepSimulation() {
    // Set simulation input voltage
    elevatorSim.setInput(leaderMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update the simulation
    elevatorSim.update(Constants.SIMULATED_LOOP_TIME);

    // var velocityRotationsPerSecond =
    //     MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond()).in(InchesPerSecond)
    //         * Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO;

    // Convert linear position to motor rotations
    double elevatorPositionMeters = elevatorSim.getPositionMeters();
    double motorRotations =
        elevatorPositionMeters / (2 * Math.PI * DRUM_RADIUS); // linear_distance / circumference

    // Convert linear velocity to motor RPM
    double elevatorVelocityMPS = elevatorSim.getVelocityMetersPerSecond();
    double motorRPM =
        (elevatorVelocityMPS / (2 * Math.PI * motorRotations)) * 60.0; // convert to RPM

    // Update the SparkMax simulation
    motorSim.iterate(motorRPM, RobotController.getBatteryVoltage(), Constants.SIMULATED_LOOP_TIME);

    // TODO: If all the conversions are set correctly, this isn't needed
    leaderMotor.getEncoder().setPosition(motorRotations);

    // Log simulation data
    Logger.recordOutput("Elevator/Sim/PositionMeters", elevatorSim.getPositionMeters());
    Logger.recordOutput(
        "Elevator/Sim/VelocityMetersPerSec", elevatorSim.getVelocityMetersPerSecond());
    Logger.recordOutput("Elevator/Sim/CurrentDrawAmps", elevatorSim.getCurrentDrawAmps());
    Logger.recordOutput("Elevator/Sim/MotorRotations", motorRotations);
    Logger.recordOutput("Elevator/Sim/MotorRPM", motorRPM);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    stepSimulation();

    // Read current elevator position from encoder
    inputs.currentHeight =
        // Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT.plus(
        Inches.of(
            leaderMotor.getEncoder().getPosition()
                * Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO);
    // );

    // Simulate limit switch based on elevator position
    boolean isBottomSwitchPressed =
        elevatorSim.getPositionMeters()
            <= Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT.in(Meters) + 0.01;

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
    Logger.recordOutput("Elevator/TargetPositionMeters", height.in(Meters));

    // Command the motor to the target position using closed-loop control
    leaderMotor
        .getClosedLoopController()
        .setReference(
            targetPosition / Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO,
            SparkBase.ControlType.kPosition);
  }

  @Override
  public void setBrakes(IdleMode idleMode) {
    var config = new SparkMaxConfig();

    config.idleMode(idleMode);

    leaderMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    Logger.recordOutput("Elevator/BrakeMode", idleMode.toString());
  }

  @Override
  public void resetHeight(Distance height) {
    // Reset the encoder position
    leaderMotor
        .getEncoder()
        .setPosition(height.in(Inches) / Constants.Elevator.ROTATIONS_TO_INCHES_CONVERSION_RATIO);
    Logger.recordOutput("Elevator/ResetHeight", height);
  }
}
