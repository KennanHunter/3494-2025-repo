package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public record SuperStructureState(ElevatorState elevatorState, ArmState armState) {
  // Create a static instance to reuse
  private static final LoggedMechanism2d mech2d =
      new LoggedMechanism2d(4, 4, new Color8Bit(Color.kWhiteSmoke));

  private static final LoggedMechanismRoot2d elevatorRoot = mech2d.getRoot("ElevatorBase", 2, 0);

  private static final LoggedMechanismLigament2d elevatorLigament =
      elevatorRoot.append(
          new LoggedMechanismLigament2d(
              "Elevator",
              Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT.in(Meters),
              90,
              3,
              new Color8Bit(Color.kBlue)));

  private static final LoggedMechanismLigament2d armLigament =
      elevatorLigament.append(
          new LoggedMechanismLigament2d("ArmStart", 0.2, 0, 3, new Color8Bit(Color.kBlue)));

  static {
    armLigament.append(
        new LoggedMechanismLigament2d("ArmIntakeBranch", 0.22, 45, 3, new Color8Bit(Color.kBlue)));

    armLigament.append(
        new LoggedMechanismLigament2d(
            "ArmAlgaeHolderBranch", 0.25, -45, 3, new Color8Bit(Color.kAqua)));
  }

  public static LoggedMechanism2d updateMechanism(SuperStructureState state) {
    double elevatorHeight =
        Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT
            .plus(state.elevatorState.height())
            .in(Meters);
    elevatorLigament.setLength(elevatorHeight);

    armLigament.setAngle(state.armState.rotation().getDegrees());

    return mech2d;
  }

  public static Pose3d[] updatePoses(SuperStructureState state) {
    Pose3d elevatorPose = Pose3d.kZero;

    // We define offsets to 0,0 in `ascope_assets/Robot_Lambda/config.json`,
    // these offset from 0,0 to the actual zeroed position on the robot
    Pose3d armOffset = new Pose3d(0, 0, 0.76, new Rotation3d(Math.toRadians(45), 0, 0));

    Transform3d armTransform =
        new Transform3d(0, 0, 0, new Rotation3d(state.armState.rotation().getRadians(), 0, 0));

    return new Pose3d[] {elevatorPose, armOffset.plus(armTransform)};
  }
}
