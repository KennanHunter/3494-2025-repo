package frc.robot.subsystems.superstructure.Arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {
  DCMotor armGearbox = DCMotor.getNeoVortex(1);

  SparkFlex armMotor = new SparkFlex(Constants.Arm.ARM_MOTOR_CAN_ID, MotorType.kBrushless);

  SparkFlexSim armMotorSim = new SparkFlexSim(armMotor, armGearbox);

  static Angle STARTING_ANGLE = Degrees.of(30);

  // Current idle mode
  private IdleMode currentIdleMode = IdleMode.kCoast;

  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          armGearbox,
          Constants.Arm.ARM_REDUCTION,
          Constants.Arm.ARM_LENGTH_TO_CENTER_OF_WHEELS.in(Meters),
          Constants.Arm.MOI,
          Constants.Arm.MIN_ANGLE.in(Radians),
          Constants.Arm.MAX_ANGLE.in(Radians),
          true,
          STARTING_ANGLE.in(Radians),
          1 / Constants.Arm.ARM_ENCODER_PULSE_PER_REV,
          0.0 // Add noise with a std-dev of 1 tick
          );

  public ArmIOSim() {
    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(IdleMode.kCoast);

    config.closedLoop.p(0.001).i(0).d(0).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    config.closedLoop.maxMotion.maxVelocity(5).maxAcceleration(5);

    config.absoluteEncoder.zeroCentered(true).positionConversionFactor(2).inverted(false);

    // Set up the motor configuration
    armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize the simulation
    armSim.setState(STARTING_ANGLE.in(Radians), 0);
    armMotor.getEncoder().setPosition(STARTING_ANGLE.in(Rotations));
  }

  private void stepSimulation() {
    armSim.setInput(armMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    armSim.update(Constants.SIMULATED_LOOP_TIME);

    // Update the motor simulation
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),
        Constants.SIMULATED_LOOP_TIME);

    // Update battery voltage simulation
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armMotorSim.getMotorCurrent()));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Run simulation step
    stepSimulation();

    inputs.armPosition = Rotation2d.fromRotations(armMotor.getAbsoluteEncoder().getPosition());
    inputs.armVelocity = RotationsPerSecond.of(armMotor.getAbsoluteEncoder().getVelocity());
    inputs.idleMode = currentIdleMode;

    Logger.recordOutput(
        "Arm/RawAbsoluteOutputRotations", armMotor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Arm/AppliedOutput", armMotor.getAppliedOutput());
    Logger.recordOutput("Arm/OutputCurrent", armMotor.getOutputCurrent());
  }

  @Override
  public void setBrakes(IdleMode mode) {
    armMotor.configure(
        new SparkFlexConfig().idleMode(mode),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void runPosition(Rotation2d newAngle) {
    Logger.recordOutput("Arm/ReferenceRotations", newAngle.getRotations());

    armMotor
        .getClosedLoopController()
        .setReference(newAngle.getRotations(), ControlType.kMAXMotionPositionControl);
  }
}
