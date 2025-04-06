package frc.robot.subsystems.superstructure.Arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  public enum ArmMode {
    Manual,
    Automatic
  }

  @AutoLog
  public static class ArmIOInputs {
    double armPosition;
    IdleMode idleMode;
    double targetPosition;
    ArmMode curretArmMode;
  }

  public default void setTargetRotation(Rotation2d targetPosition) {}

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setBrakes(IdleMode neutralMode) {}
}
