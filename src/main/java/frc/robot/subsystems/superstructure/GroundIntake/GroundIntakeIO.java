package frc.robot.subsystems.superstructure.GroundIntake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngularVelocityUnit;
import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
  @AutoLog
  public static class GroundIntakeIOInputs {
    public Rotation2d pivotPosition;

    public double frontRollerCurrent;
    public double backRollerCurrent;

    public IdleMode idleMode;
  }

  public default void runPivot(Rotation2d angle) {}

  public default void runIntakeSpeed(AngularVelocityUnit speed) {}

  public default void updateInputs(GroundIntakeIOInputs inputs) {}

  public default void setBrakes(IdleMode neutralMode) {}
}
