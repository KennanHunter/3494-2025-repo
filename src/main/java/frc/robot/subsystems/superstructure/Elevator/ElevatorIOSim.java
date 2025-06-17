package frc.robot.subsystems.superstructure.Elevator;

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
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  // Simulation parameters
  private static final double ELEVATOR_DRUM_RADIUS_METERS = 0.02;
  private static final double ELEVATOR_GEARING = 20.0;
  private static final double CARRIAGE_MASS_KG = 4.0;
  private static final double MIN_HEIGHT_METERS = 0.0;
  private static final double MAX_HEIGHT_METERS = 1.5;
  private static final double POSITION_CONVERSION_FACTOR = 10.0;

  // SparkMax hardware objects
  private final SparkMax leaderMotor;
  private final SparkMax followerMotor;
  private final SparkMaxConfig leaderConfig;
  private final SparkMaxConfig followerConfig;
  private final DCMotor elevatorGearbox;

  // Simulation objects
  private final SparkMaxSim leaderMotorSim;
  private final SparkMaxSim followerMotorSim;
  private final ElevatorSim elevatorSim;

  // State tracking
  private boolean isBottomSensorTripped = false;

  public ElevatorIOSim() {
    // Create SparkMax motors - use the same IDs as real hardware
    leaderMotor = new SparkMax(Constants.Elevator.leaderMotor, MotorType.kBrushless);
    followerMotor = new SparkMax(Constants.Elevator.followerMotor, MotorType.kBrushless);

    // Configure motors the same way as real hardware
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

    elevatorGearbox = DCMotor.getNEO(2);

    // Setup simulation objects
    leaderMotorSim = new SparkMaxSim(leaderMotor, elevatorGearbox);
    followerMotorSim = new SparkMaxSim(followerMotor, elevatorGearbox);

    // Create elevator physics simulation
    elevatorSim =
        new ElevatorSim(
            elevatorGearbox, // 2 NEO motors
            ELEVATOR_GEARING,
            CARRIAGE_MASS_KG,
            ELEVATOR_DRUM_RADIUS_METERS,
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            true, // Simulate gravity
            0.0 // Starting position
            );
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Update simulation physics
    elevatorSim.update(Constants.SIMULATED_LOOP_TIME);

    // Get voltage from SparkMax controller
    double motorVoltage = leaderMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();

    // Set elevator sim input
    elevatorSim.setInputVoltage(motorVoltage);

    // Update SparkMax simulation with physics model
    leaderMotorSim.setMotorCurrent(
        elevatorSim.getCurrentDrawAmps() / 2); // Split current between motors
    followerMotorSim.setMotorCurrent(elevatorSim.getCurrentDrawAmps() / 2);

    // Update SparkMax encoder using physics model position
    double positionTicks = elevatorSim.getPositionMeters() * POSITION_CONVERSION_FACTOR;
    double velocityTicksPerSecond =
        elevatorSim.getVelocityMetersPerSecond() * POSITION_CONVERSION_FACTOR;

    leaderMotorSim.setPosition(positionTicks);
    leaderMotorSim.setVelocity(velocityTicksPerSecond);

    // Update battery simulation
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    // Update sensor state
    isBottomSensorTripped = (elevatorSim.getPositionMeters() < 0.01);

    inputs.sensorState =
        isBottomSensorTripped ? ElevatorSensorState.BOTTOM : ElevatorSensorState.UP;
    inputs.currentHeight =
        Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT.plus(
            Meters.of(elevatorSim.getPositionMeters()));
  }

  @Override
  public void runElevatorHeight(Distance height) {
    Distance computedElevatorHeight =
        height.minus(Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT);

    Logger.recordOutput("Elevator/computedElevatorMeters", computedElevatorHeight.in(Meters));

    leaderMotor
        .getClosedLoopController()
        .setReference(computedElevatorHeight.in(Meters), SparkBase.ControlType.kPosition);
  }

  @Override
  public void setBrakes(IdleMode idleMode) {
    leaderConfig.idleMode(idleMode);
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void resetPosition(double position) {
    // Reset both the motor encoder and the physics simulation
    leaderMotor.getEncoder().setPosition(position / POSITION_CONVERSION_FACTOR);
    elevatorSim.setState(
        position / POSITION_CONVERSION_FACTOR, elevatorSim.getVelocityMetersPerSecond());
  }

  @Override
  public void setPIDlimits(double lowerBound, double upperBound) {
    leaderConfig.closedLoop.outputRange(lowerBound, upperBound);
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
