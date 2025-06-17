package frc.robot.subsystems.superstructure.Arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    Rotation2d armPosition;
    IdleMode idleMode;
    AngularVelocity armVelocity;
  }

  public default void runPosition(Rotation2d angle) {}

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setBrakes(IdleMode neutralMode) {}
}
