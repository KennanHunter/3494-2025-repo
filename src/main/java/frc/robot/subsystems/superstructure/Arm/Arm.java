package frc.robot.subsystems.superstructure.Arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public class Arm extends SubsystemBase {
  private final ArmIO armIO;
  private ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();

  // Mechanism2d visualization
  public final LoggedMechanism2d mech2d = new LoggedMechanism2d(60, 60);

  public Angle ACCEPTABLE_ANGLE_ERROR = Degrees.of(15);

  private ArmState target;

  public Arm(ArmIO armIO) {
    this.armIO = armIO;
  }

  public Optional<ArmState> getState() {
    if (armIOInputs.armPosition == null) {
      return Optional.empty();
    }

    return Optional.of(new ArmState(armIOInputs.armPosition));
  }

  public void setTargetState(ArmState armState) {
    target = armState;

    this.armIO.runPosition(armState.rotation());
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armIOInputs);
    Logger.processInputs("Arm", armIOInputs);

    getState()
        .ifPresent(
            (val) -> {
              Logger.recordOutput("Arm/CurrentRotations", val);
            });

    Logger.recordOutput("Arm/Target", this.target);
  }

  public boolean isAtTarget() {
    if (this.target == null) return false;

    return this.target.rotation().minus(this.armIOInputs.armPosition).getDegrees()
        <= ACCEPTABLE_ANGLE_ERROR.in(Degrees);
  }
}
