package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;

//
public class WheelOffsetCalculator extends Command {
  private final Drive drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WheelOffsetCalculator(Drive drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  @Override
  public void execute() {
    Rotation2d[] positions = drive.getRawTurnEncoderPositions();

    if (positions == null || positions.length < 4) {
      System.err.println("Error: Encoder positions are invalid.");
      return;
    }

    DecimalFormat df = new DecimalFormat("#.#");

    System.out.println(
        "public static final double FRONT_LEFT_OFFSET = Math.toRadians("
            + df.format(positions[0].getDegrees())
            + ");");
    System.out.println(
        "public static final double FRONT_RIGHT_OFFSET = Math.toRadians("
            + df.format(positions[1].getDegrees())
            + ");");
    System.out.println(
        "public static final double BACK_LEFT_OFFSET = Math.toRadians("
            + df.format(positions[2].getDegrees())
            + ");");
    System.out.println(
        "public static final double BACK_RIGHT_OFFSET = Math.toRadians("
            + df.format(positions[3].getDegrees())
            + ");");

    System.out.println("\n");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
