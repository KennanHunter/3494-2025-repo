package frc.robot.subsystems.superstructure.Arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
    public double armVelocity;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setBrakes(IdleMode neutralMode) {}
}
