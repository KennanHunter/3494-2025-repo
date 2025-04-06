package frc.robot.subsystems.superstructure.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public class Arm extends SubsystemBase {
  private final ArmIO armIO;
  private ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();

  // Mechanism2d visualization
  public final LoggedMechanism2d mech2d = new LoggedMechanism2d(60, 60);

  public Arm(ArmIO armIO) {
    this.armIO = armIO;
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armIOInputs);
    Logger.processInputs("Arm", armIOInputs);
  }
}
