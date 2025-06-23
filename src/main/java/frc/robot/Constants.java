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

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import au.grapplerobotics.MitoCANdria;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.LoggedTunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // We always want tuning mode in Sim, use `tuningModeOnRealField` for real deploys
  private static final boolean isRealFieldTuningModeEnabled = false;
  public static final boolean tuningMode = RobotBase.isSimulation() || isRealFieldTuningModeEnabled;

  // Should be pretty close to real
  public static final double SIMULATED_LOOP_TIME = 0.02;

  public static final int POWER_DISTRIBUTION_PANEL_CAN_ID = 28;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Mitocandria {
    public static final int MITOCANDRIA_CAN_ID = 50;

    public static final int LIMELIGHT_CHANNEL = MitoCANdria.MITOCANDRIA_CHANNEL_5VB;
    public static final int PIGEON_CHANNEL = MitoCANdria.MITOCANDRIA_CHANNEL_ADJ;
  }

  public static class Presets {}

  public static class OI {
    public static int PRIMARY_CONTROLLER_PORT = 0;
  }

  public static class Elevator {
    public static int bottomMagSensorDIO = 9;
    public static int leaderMotor = 12;
    public static int followerMotor = 13;

    public static final Distance PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT = Inches.of(29.5);
    public static final Distance PHYSICAL_ELEVATOR_TOP_HEIGHT_MEASUREMENT = Inches.of(49);

    // TODO: Figure out how to make this work with Measure<PerUnit<AngleUnit, DistanceUnit>> or
    // Per<Angle, Distance> or similar
    public static final double ROTATIONS_TO_INCHES_CONVERSION_RATIO = (4.0 / 9.0);

    public static final Mass CARRIAGE_MASS = Kilograms.of(4.0);

    /**
     * @param percentage value from 0 to 1
     * @return distance
     */
    public static Distance positionFromPercentageDecimal(double percentage) {
      assert percentage >= 0;
      assert percentage <= 1;

      Double displacement =
          PHYSICAL_ELEVATOR_TOP_HEIGHT_MEASUREMENT.in(Meters)
              - PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT.in(Meters);

      return PHYSICAL_ELEVATOR_BOTTOM_HEIGHT_MEASUREMENT.plus(Meters.of(displacement * percentage));
    }
  }

  public static class Arm {
    public static int ARM_MOTOR_CAN_ID = 15;

    public static final Angle MIN_ANGLE = Degrees.of(-80);
    public static final Angle MAX_ANGLE = Degrees.of(80);
    public static final Distance ARM_LENGTH_TO_CENTER_OF_WHEELS = Meters.of(0.4);

    public static final double ARM_REDUCTION = 22 / 3;

    public static final double MOI =
        SingleJointedArmSim.estimateMOI(ARM_LENGTH_TO_CENTER_OF_WHEELS.in(Meters), 2);

    public static final int ARM_ENCODER_PULSE_PER_REV = 4096;

    public static final LoggedTunableNumber p = new LoggedTunableNumber("Arm/Tuning/P", 4.0);
    public static final LoggedTunableNumber i = new LoggedTunableNumber("Arm/Tuning/I", 0.0);
    public static final LoggedTunableNumber d = new LoggedTunableNumber("Arm/Tuning/D", 0.0);
    public static final LoggedTunableNumber bound =
        new LoggedTunableNumber("Arm/Tuning/Bounds", 0.6);
  }

  public static class Intake {
    public static int INTAKE_MOTOR_CAN_ID = 14;
  }

  public static class Climber {
    public static int CLIMBER_MOTOR_CAN_ID = 6;
  }

  public static class Drivetrain {
    public static double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }

    public static final double trackWidthX = 0.5222; // TODO: random Number
    public static final double trackWidthY = 0.574675;

    public static final double maxLinearVelocity = 4.5; // TODO: I made this number up
    public static final double maxLinearAcceleration = 20.0; // TODO: I made this number up
    public static final double maxAngularVelocity = 6;
    public static final double maxAngularAcceleration = 12;

    public static final int PIGEON_PORT = 52;
    public static final int FRONT_LEFT_DRIVE_ID = 18; // 18
    public static final int FRONT_LEFT_STEER_ID = 16; // 16
    public static final int FRONT_LEFT_TURN_ENCODER_ID = 3; // 3
    public static final double FRONT_LEFT_OFFSET = Math.toRadians(204.4);

    public static final int FRONT_RIGHT_DRIVE_ID = 19; // 19
    public static final int FRONT_RIGHT_STEER_ID = 17; // 17
    public static final int FRONT_RIGHT_TURN_ENCODER_ID = 2; // 2

    public static final double FRONT_RIGHT_OFFSET = Math.toRadians(237.9);

    public static final int BACK_LEFT_DRIVE_ID = 30; // 30
    public static final int BACK_LEFT_STEER_ID = 2; // 2
    public static final int BACK_LEFT_TURN_ENCODER_ID = 1; // 1
    public static final double BACK_LEFT_OFFSET = Math.toRadians(264.5);

    public static final int BACK_RIGHT_DRIVE_ID = 1; // 1
    public static final int BACK_RIGHT_STEER_ID = 3; // 3
    public static final int BACK_RIGHT_TURN_ENCODER_ID = 0; // 0
    public static final double BACK_RIGHT_OFFSET = Math.toRadians(146.8);
  }
}
