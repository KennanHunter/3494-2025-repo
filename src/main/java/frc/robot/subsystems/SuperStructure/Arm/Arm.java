package frc.robot.subsystems.SuperStructure.Arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO armIO;

  private ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO armIO) {
    this.armIO = armIO;
  }

  public void setBrakes(IdleMode neutralMode) {
    armIO.setBrakes(neutralMode);
  }

  public void setTargetAngle(Rotation2d rotation) {
    armIO.setTargetRotation(rotation);
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armIOInputs);
    Logger.processInputs("Arm", armIOInputs);
  }

  public double getAbsoluteTicks() {
    return armIOInputs.armPosition;
  }

  public double getTargetPosition() {
    return armIOInputs.targetPosition;
  }
}
