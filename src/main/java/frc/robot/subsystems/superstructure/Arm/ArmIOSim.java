package frc.robot.subsystems.superstructure.Arm;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {
  DCMotor armGearbox = DCMotor.getNeoVortex(1);

  SparkFlex armMotor = new SparkFlex(Constants.Arm.ARM_MOTOR_CAN_ID, MotorType.kBrushless);

  SparkFlexSim armMotorSim = new SparkFlexSim(armMotor, armGearbox);

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
          0,
          1 / Constants.Arm.ARM_ENCODER_PULSE_PER_REV,
          0.0 // Add noise with a std-dev of 1 tick
          );

  public ArmIOSim() {
    SparkBaseConfig config = new SparkFlexConfig().idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    // Set up the motor configuration
    armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    double startingPosDegrees = 0;

    // Initialize the simulation
    armSim.setState(Math.toRadians(startingPosDegrees), 0.2);
    armMotor.getEncoder().setPosition(startingPosDegrees);
  }

  private void stepSimulation() {
    armSim.update(Constants.SIMULATED_LOOP_TIME);

    // Update the motor simulation
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),
        Constants.SIMULATED_LOOP_TIME);

    // Update battery voltage simulation
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armMotorSim.getMotorCurrent()));

    // Update encoder position from the arm simulation
    armMotorSim
        .getAbsoluteEncoderSim()
        .setPosition(Units.radiansToRotations(armSim.getAngleRads()));
    armMotorSim
        .getAbsoluteEncoderSim()
        .setVelocity(Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec()));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Run simulation step
    stepSimulation();

    // Update the inputs
    inputs.armPosition = Rotation.of(armMotor.getEncoder().getPosition());
    inputs.armVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(armMotor.getEncoder().getVelocity());
    // inputs.armAppliedVolts = armMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    // inputs.armCurrentAmps = armMotorSim.getMotorCurrent();
    inputs.idleMode = currentIdleMode;
  }

  @Override
  public void setBrakes(IdleMode mode) {
    armMotor.configure(
        new SparkFlexConfig().idleMode(mode),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }
}
