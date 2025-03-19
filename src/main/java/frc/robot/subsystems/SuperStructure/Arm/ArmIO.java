package frc.robot.subsystems.SuperStructure.Arm;

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
  }

  double getArmPosition();

  void setTargetPosition(double targetPosition);

  void updateInputs(ArmIOInputs inputs);

  void setBrakes(IdleMode neutralMode);
}
