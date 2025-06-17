package frc.robot.subsystems.superstructure.Arm;

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
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ArmIOSpark implements ArmIO {
  private final SparkFlex motor;
  public final LoggedTunableNumber p = new LoggedTunableNumber("Arm/P", 6.0);
  public final LoggedTunableNumber i = new LoggedTunableNumber("Arm/I", 0.0);
  public final LoggedTunableNumber d = new LoggedTunableNumber("Arm/D", 0.0);
  public final LoggedTunableNumber bound = new LoggedTunableNumber("Arm/Bounds", 0.6);

  public ArmIOSpark() {
    motor = new SparkFlex(Constants.Arm.ARM_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(IdleMode.kCoast);

    config
        .closedLoop
        .p(p.get())
        .i(i.get())
        .d(d.get())
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    config.closedLoop.maxMotion.maxVelocity(120).maxAcceleration(60);

    config
        .absoluteEncoder
        .zeroOffset(0.2)
        .zeroCentered(true)
        .positionConversionFactor(2)
        .inverted(false);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    configurePID();

    Logger.recordOutput("Arm/RawAbsoluteOutputRotations", motor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Arm/AppliedOutput", motor.getAppliedOutput());
    Logger.recordOutput("Arm/OutputCurrent", motor.getOutputCurrent());

    inputs.armPosition = Rotation2d.fromRotations(motor.getAbsoluteEncoder().getPosition());
  }

  public void configurePID() {
    SparkFlexConfig config = new SparkFlexConfig();

    if (!p.hasChanged(hashCode())
        && !i.hasChanged(hashCode())
        && !d.hasChanged(hashCode())
        && !bound.hasChanged(hashCode())) return;

    ClosedLoopConfig newConfig = new ClosedLoopConfig();

    newConfig.p(p.get());
    newConfig.i(i.get());
    newConfig.d(d.get());
    newConfig.outputRange(bound.get(), -bound.get());

    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
