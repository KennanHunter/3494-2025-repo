package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperStructureVisualizer {
  private final LoggedMechanism2d mech2d;
  private final LoggedMechanismRoot2d elevatorRoot;
  private final LoggedMechanismLigament2d elevatorLigament;
  private final LoggedMechanismLigament2d armLigament;

  public SuperStructureVisualizer(Color8Bit mainColor, Color8Bit accentColor) {
    // Initialize the 2D mechanism
    this.mech2d = new LoggedMechanism2d(4, 4, new Color8Bit(Color.kWhiteSmoke));
    this.elevatorRoot = mech2d.getRoot("ElevatorBase", 2, 0);

    this.elevatorLigament =
        elevatorRoot.append(
            new LoggedMechanismLigament2d(
                "Elevator",
                Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT.in(Meters),
                90,
                3,
                new Color8Bit(
                    (int) Math.round(mainColor.red * 0.6),
                    (int) Math.round(mainColor.green * 0.6),
                    (int) Math.round(mainColor.blue * 0.6))));

    this.armLigament =
        elevatorLigament.append(new LoggedMechanismLigament2d("ArmStart", 0.2, 0, 3, mainColor));

    // Add arm branches
    armLigament.append(new LoggedMechanismLigament2d("ArmIntakeBranch", 0.22, 45, 3, mainColor));

    armLigament.append(
        new LoggedMechanismLigament2d("ArmAlgaeHolderBranch", 0.25, -45, 3, accentColor));
  }

  public LoggedMechanism2d updateMechanism(SuperStructureState state) {
    double elevatorHeight = state.elevatorState().height().in(Meters);

    elevatorLigament.setLength(elevatorHeight);

    armLigament.setAngle(state.armState().rotation().getDegrees());

    return mech2d;
  }

  public Pose3d[] updatePoses(SuperStructureState state) {
    Pose3d elevatorPose =
        new Pose3d(
            0.0,
            0.0,
            state
                .elevatorState()
                .height()
                .minus(Constants.Elevator.PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT)
                .in(Meters),
            Rotation3d.kZero);

    // We define offsets to 0,0 in `ascope_assets/Robot_Lambda/config.json`,
    // these offset from 0,0 to the actual zeroed position on the robot
    Pose3d armOffset =
        new Pose3d(
            0,
            0,
            0.015 + state.elevatorState().height().in(Meters),
            new Rotation3d(Math.toRadians(125), 0, 0));

    Transform3d armTransform =
        new Transform3d(0, 0, 0, new Rotation3d(state.armState().rotation().getRadians(), 0, 0));

    return new Pose3d[] {elevatorPose, armOffset.plus(armTransform)};
  }
}
