package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class JoyStickDriveCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final double DEADBAND = 0.1;

  private Translation2d linearVelocity;
  private Translation2d pastLinearVelocity;
  private double timeAtLastRead;
  private final Timer timer = new Timer();

  public JoyStickDriveCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    // Apply deadband and get magnitude
    double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
    double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND);
    double linearMagnitude = Math.hypot(x, y);

    if (y == 0) {
      y = Double.MIN_VALUE;
    }

    // Get direction and apply deadband to rotation
    Rotation2d linearDirection = new Rotation2d(x, y);
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

    // Square values for smoother control
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calculate linear velocity
    linearVelocity = new Translation2d(linearMagnitude, linearDirection);

    boolean isFieldFlipped =
        DriverStation.getAlliance()
            .filter(alliance -> alliance == DriverStation.Alliance.Red)
            .isPresent();

    // Apply to drive
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec(),
            isFieldFlipped
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()));

    // Log velocity data
    logVelocityData();

    // Store for next iteration
    pastLinearVelocity = linearVelocity;
    timeAtLastRead = timer.get();
  }

  private void logVelocityData() {
    if (linearVelocity != null && pastLinearVelocity != null) {
      // Calculate current velocity from drive
      var twist = drive.getfieldVelocity();
      double currentVelocity = Math.hypot(twist.dx, twist.dy);

      // Calculate target velocity
      double targetVelocity = linearVelocity.getNorm() * drive.getMaxLinearSpeedMetersPerSec();

      // Calculate acceleration
      double acceleration =
          (linearVelocity.getNorm() - currentVelocity) / (timer.get() - timeAtLastRead);

      // Log data
      Logger.recordOutput("Drive/CurrentLinearVelocity", currentVelocity);
      Logger.recordOutput("Drive/TargetLinearVelocity", targetVelocity);
      Logger.recordOutput("Drive/LinearAcceleration", acceleration);
    }
  }
}
