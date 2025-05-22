package frc.robot.subsystems.superstructure.Arm;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
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

  public Optional<ArmState> getState() {
    if (armIOInputs.armPosition == null) {
      return Optional.empty();
    }

    return Optional.of(
        new ArmState(Rotation2d.fromRotations(armIOInputs.armPosition.in(Rotation))));
  }

  public void setState(ArmState newState) {
    armIOInputs.armPosition = Rotation.of(newState.rotation().getRotations());
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armIOInputs);
    Logger.processInputs("Arm", armIOInputs);
  }
}
