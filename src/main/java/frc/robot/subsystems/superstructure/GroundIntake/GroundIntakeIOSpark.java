package frc.robot.subsystems.superstructure.GroundIntake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class GroundIntakeIOSpark implements GroundIntakeIO {
  private final SparkFlex pivotMotor;
  private final SparkFlex frontIntakeMotor;
  private final SparkFlex backIntakeMotor;

  public GroundIntakeIOSpark() {
    pivotMotor = new SparkFlex(Constants.GroundIntake.PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
    frontIntakeMotor =
        new SparkFlex(Constants.GroundIntake.FRONT_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    backIntakeMotor =
        new SparkFlex(Constants.GroundIntake.BACK_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontIntakeMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backIntakeMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {
    inputs.pivotPosition = Rotation2d.fromRotations(pivotMotor.getAbsoluteEncoder().getPosition());

    inputs.frontRollerCurrent = frontIntakeMotor.getOutputCurrent();
    inputs.backRollerCurrent = backIntakeMotor.getOutputCurrent();
  }
}
