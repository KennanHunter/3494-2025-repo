package frc.robot.subsystems.superstructure.Arm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ArmIOSpark implements ArmIO {
  private final SparkFlex motor;
  private final SparkFlexConfig config;
  private final SparkClosedLoopController controller;

  public final LoggedTunableNumber p = new LoggedTunableNumber("Arm/P", 6.0);
  public final LoggedTunableNumber i = new LoggedTunableNumber("Arm/I", 0.0);
  public final LoggedTunableNumber d = new LoggedTunableNumber("Arm/D", 0.0);
  public final LoggedTunableNumber lowerBound = new LoggedTunableNumber("Arm/LowerBound", -0.5);
  public final LoggedTunableNumber upperBound = new LoggedTunableNumber("Arm/UpperBound", 0.5);

  public ArmIOSpark() {
    motor = new SparkFlex(Constants.Arm.ARM_MOTOR_CAN_ID, MotorType.kBrushless);
    config = new SparkFlexConfig();
    controller = motor.getClosedLoopController();

    config.idleMode(IdleMode.kCoast);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setTargetRotation(Rotation2d targetPosition) {
    // controller.setReference(
    //     targetPosition + Constants.Presets.globalArmOffset,
    //     SparkMax.ControlType.kPosition,
    //     ClosedLoopSlot.kSlot0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // TODO: Is it smart to do stateful actions in updateInputs? Is it a bad mix of functionality?
    configurePID();

    // inputs.armPosition = getArmPosition();

  }

  public void configurePID() {
    ClosedLoopConfig newConfig = new ClosedLoopConfig();

    if (p.hasChanged(hashCode())) newConfig.p(p.get());
    if (i.hasChanged(hashCode())) newConfig.i(i.get());
    if (d.hasChanged(hashCode())) newConfig.d(d.get());
    if (lowerBound.hasChanged(hashCode()) || upperBound.hasChanged(hashCode()))
      newConfig.outputRange(lowerBound.get(), upperBound.get());

    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setBrakes(IdleMode neutralMode) {
    config.idleMode(neutralMode);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}
