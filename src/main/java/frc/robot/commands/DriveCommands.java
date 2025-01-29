// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.AutoAlignController;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private DriveMode currentDriveMode = DriveMode.NORMAL_TELEOP;
  private static AutoAlignController autoAlignController = null;
  private static ChassisSpeeds desiredSpeeds = null;

  // 9c
  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped = false;
          //   DriverStation.getAlliance().isPresent()
          //       // We're flipping at Blue instead of Red (which was 6328 default)
          //       && DriverStation.getAlliance().get() == Alliance.Blue;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command autoAlign(Drive drive, boolean leftSide) {
    System.out.println("REUESTED--------------------");
    Supplier<Pose2d> ampAlignedPose =
        () -> {
          Pose2d ampCenterRotated =
              new Pose2d(Constants.Field.ampCenter, new Rotation2d(-Math.PI / 2.0));
          double distance =
              drive.getPose().getTranslation().getDistance(ampCenterRotated.getTranslation());
          double offsetT = MathUtil.clamp((distance - 0.3) / 2.5, 0.0, 1.0);
          return ampCenterRotated.transformBy(
              new Transform2d(offsetT * 1.75, 0.0, new Rotation2d()));
        };
    Supplier<Pose2d> onTheFly = AutoAlignDesitationDeterminer.destination(drive.getPose(), leftSide);
    autoAlignController =
        new AutoAlignController(
            drive,
            onTheFly,//ampAlignedPose,
            () -> {
              return new Translation2d();
            },
            false);

    return Commands.runOnce(
        () -> {
          desiredSpeeds = autoAlignController.update(drive);
          drive.runVelocity(desiredSpeeds);
        },
        drive);
  }

  public void setDriveMode(DriveMode newDriveMode) {
    currentDriveMode = newDriveMode;
  }
}
