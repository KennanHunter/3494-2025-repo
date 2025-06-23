package frc.robot.subsystems.superstructure.Arm;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {
  private final SparkFlex motor;

  // Simulation components
  private final DCMotor armMotorModel = DCMotor.getNEO(1);
  private final SparkFlexSim motorSim;
  private final SingleJointedArmSim armSim;

  public ArmIOSim() {
    motor = new SparkFlex(Constants.Arm.ARM_MOTOR_CAN_ID, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(IdleMode.kCoast);

    config
        .closedLoop
        .p(Constants.Arm.p.get())
        .i(Constants.Arm.i.get())
        .d(Constants.Arm.d.get())
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    config.closedLoop.maxMotion.maxVelocity(120).maxAcceleration(60);

    config
        .absoluteEncoder
        .zeroOffset(0.2)
        .zeroCentered(true)
        .positionConversionFactor(2)
        .inverted(false);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize simulation
    motorSim = new SparkFlexSim(motor, armMotorModel);
    armSim =
        new SingleJointedArmSim(
            armMotorModel,
            Constants.Arm.ARM_REDUCTION,
            Constants.Arm.MOI,
            Constants.Arm.ARM_LENGTH_TO_CENTER_OF_WHEELS.in(Meters),
            Constants.Arm.MIN_ANGLE.in(Radians),
            Constants.Arm.MAX_ANGLE.in(Radians),
            true,
            0.0);
  }

  private void stepSimulation() {
    // Set simulation input voltage
    armSim.setInput(motor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update the simulation
    armSim.update(Constants.SIMULATED_LOOP_TIME);

    // Update the SparkFlex simulation
    motorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            armSim.getVelocityRadPerSec() * Constants.Arm.ARM_REDUCTION),
        RobotController.getBatteryVoltage(),
        Constants.SIMULATED_LOOP_TIME);

    // Log simulation data
    Logger.recordOutput("Arm/Sim/AngleRads", armSim.getAngleRads());
    Logger.recordOutput("Arm/Sim/VelocityRadPerSec", armSim.getVelocityRadPerSec());
    Logger.recordOutput("Arm/Sim/CurrentDrawAmps", armSim.getCurrentDrawAmps());
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    stepSimulation();
    configurePID();

    Logger.recordOutput("Arm/RawAbsoluteOutputRotations", motor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Arm/AppliedOutput", motor.getAppliedOutput());
    Logger.recordOutput("Arm/OutputCurrent", motor.getOutputCurrent());

    inputs.armPosition = Rotation2d.fromRotations(motor.getAbsoluteEncoder().getPosition());
  }

  public void configurePID() {
    if (!Constants.Arm.p.hasChanged(hashCode())
        && !Constants.Arm.i.hasChanged(hashCode())
        && !Constants.Arm.d.hasChanged(hashCode())
        && !Constants.Arm.bound.hasChanged(hashCode())) return;

    ClosedLoopConfig newConfig = new ClosedLoopConfig();

    newConfig.p(Constants.Arm.p.get());
    newConfig.i(Constants.Arm.i.get());
    newConfig.d(Constants.Arm.d.get());
    newConfig.outputRange(Constants.Arm.bound.get(), -Constants.Arm.bound.get());

    motor.configure(
        new SparkFlexConfig().apply(newConfig),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void setBrakes(IdleMode neutralMode) {
    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(neutralMode);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void runPosition(Rotation2d newAngle) {
    Logger.recordOutput("Arm/ReferenceRotations", newAngle.getRotations());

    motor
        .getClosedLoopController()
        .setReference(newAngle.getRotations(), ControlType.kMAXMotionPositionControl);
  }
}
