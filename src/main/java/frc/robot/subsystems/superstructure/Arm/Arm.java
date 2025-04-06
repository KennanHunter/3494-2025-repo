package frc.robot.subsystems.superstructure.Arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.Arm.ArmIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Arm extends SubsystemBase {
  private final ArmIO armIO;
  private ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();

  // Mechanism2d visualization
  public final LoggedMechanism2d mech2d = new LoggedMechanism2d(60, 60);
  private final LoggedMechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  private final LoggedMechanismLigament2d armLigament;
  private final LoggedMechanismLigament2d targetArmLigament;

  public Arm(ArmIO armIO) {
    this.armIO = armIO;

    // Initialize the arm visualization with a fixed length and color
    armLigament =
        armPivot.append(
            new LoggedMechanismLigament2d("Arm", 20, 0, 6, new Color8Bit(Color.kYellow)));

    // Add a second ligament for the target position
    targetArmLigament =
        armPivot.append(
            new LoggedMechanismLigament2d("TargetArm", 20, 0, 4, new Color8Bit(Color.kGreen)));

    // Publish the mechanism to SmartDashboard
    Logger.recordOutput("Mechanisms/Arm", mech2d);
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

    // Update the mechanism visualization with the current arm position
    // Note: Mechanism2d uses counterclockwise-positive angles in degrees
    double armAngleDegrees = Math.toDegrees(armIOInputs.armPosition);
    armLigament.setAngle(armAngleDegrees);

    // Update the target arm visualization
    double targetAngleDegrees = Math.toDegrees(armIOInputs.targetPosition);
    targetArmLigament.setAngle(targetAngleDegrees);
  }

  public double getAbsoluteTicks() {
    return armIOInputs.armPosition;
  }

  public double getTargetPosition() {
    return armIOInputs.targetPosition;
  }
}
