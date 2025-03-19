package frc.robot.subsystems.SuperStructure.Arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIOSpark armIO;

  private ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIOSpark armIO) {
    this.armIO = armIO;
  }

  public void setBrakes(IdleMode neutralMode) {
    armIO.setBrakes(neutralMode);
  }

  public void setTargetAngle(double ticks, double arbFFVoltage) {
    double targetPosition = ticks;
    armIO.setTargetPosition(targetPosition);
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armIOInputs);
    Logger.processInputs("Arm", armIOInputs);
    armIO.configurePID();
  }

  public double getAbsoluteTicks() {
    return armIOInputs.armPosition;
  }

  public double getTargetPosition() {
    return armIOInputs.targetPosition;
  }
}
