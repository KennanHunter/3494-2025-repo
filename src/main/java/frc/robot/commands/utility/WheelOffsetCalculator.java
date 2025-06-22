package frc.robot.commands.utility;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.util.ArrayList;

public class WheelOffsetCalculator extends Command {
  private final Drive drive;

  /**
   * Command to calculate and log the wheel offsets for a swerve drive system.
   *
   * @param drive The Drive subsystem that contains the swerve drive logic.
   */
  public WheelOffsetCalculator(Drive drive) {
    this.drive = drive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    ArrayList<Rotation2d> positions = drive.getRawTurnEncoderPositions();

    if (positions == null || positions.size() < 4) {
      System.err.println("Error: Encoder positions are invalid.");
      return;
    }

    DecimalFormat df = new DecimalFormat("#.#");

    // TODO: Figure out why color codes don't work in VSCode
    // String PURPLE = "\u001B[35m";
    // String ANSI_RESET = "\u001B[0m";
    // For now, we will use empty strings for color codes
    String PURPLE = "";
    String ANSI_RESET = "";

    System.out.println(
        PURPLE
            + "public static final double FRONT_LEFT_OFFSET = Math.toRadians("
            + df.format(positions.get(0).getDegrees())
            + ");"
            + ANSI_RESET);
    System.out.println(
        PURPLE
            + "public static final double FRONT_RIGHT_OFFSET = Math.toRadians("
            + df.format(positions.get(1).getDegrees())
            + ");"
            + ANSI_RESET);
    System.out.println(
        PURPLE
            + "public static final double BACK_LEFT_OFFSET = Math.toRadians("
            + df.format(positions.get(2).getDegrees())
            + ");"
            + ANSI_RESET);
    System.out.println(
        PURPLE
            + "public static final double BACK_RIGHT_OFFSET = Math.toRadians("
            + df.format(positions.get(3).getDegrees())
            + ");"
            + ANSI_RESET);

    System.out.println("\n");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
