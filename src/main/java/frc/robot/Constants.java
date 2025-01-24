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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
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
        public static final double FRONT_LEFT_OFFSET = Math.toRadians(66.0);

        public static final int FRONT_RIGHT_DRIVE_ID = 19; // 19
        public static final int FRONT_RIGHT_STEER_ID = 17; // 17
        public static final int FRONT_RIGHT_TURN_ENCODER_ID = 2; // 2

        public static final double FRONT_RIGHT_OFFSET =
                Math.toRadians(180 - 28); // Kinda ok for now

        public static final int BACK_LEFT_DRIVE_ID = 30; // 30
        public static final int BACK_LEFT_STEER_ID = 2; // 2
        public static final int BACK_LEFT_TURN_ENCODER_ID = 1; // 1
        public static final double BACK_LEFT_OFFSET = Math.toRadians(18);

        public static final int BACK_RIGHT_DRIVE_ID = 1; // 1
        public static final int BACK_RIGHT_STEER_ID = 3; // 3
        public static final int BACK_RIGHT_TURN_ENCODER_ID = 0; // 0
        public static final double BACK_RIGHT_OFFSET = Math.toRadians(180 + 45.0);
    }

    public static class Field {
        public static final double fieldLength = Units.inchesToMeters(651.223);
        public static final double fieldWidth = Units.inchesToMeters(323.277);
        public static final Translation2d ampCenter =
                new Translation2d(Units.inchesToMeters(72.455), fieldWidth);
    }
}

/*The bad constatns that kinda work
* public static final int PIGEON_PORT = 52;
   public static final int FRONT_LEFT_DRIVE_ID = 30; // 18
   public static final int FRONT_LEFT_STEER_ID = 2; // 16
   public static final int FRONT_LEFT_TURN_ENCODER_ID = 1; // 3
   public static final double FRONT_LEFT_OFFSET =
       Math.toRadians(-253.2 + 180 - 5.6); // Math.toRadians(-18.0)

   public static final int FRONT_RIGHT_DRIVE_ID = 18; // 19
   public static final int FRONT_RIGHT_STEER_ID = 16; // 17
   public static final int FRONT_RIGHT_TURN_ENCODER_ID = 3; // 2
   public static final double FRONT_RIGHT_OFFSET =
       Math.toRadians(-18.0); // Math.toRadians(64.3993);

   public static final int BACK_LEFT_DRIVE_ID = 1; // 30
   public static final int BACK_LEFT_STEER_ID = 3; // 2
   public static final int BACK_LEFT_TURN_ENCODER_ID = 0; // 1
   public static final double BACK_LEFT_OFFSET =
       Math.toRadians(-46 + 180); // Math.toRadians(-253.2 + 180 - 5.6)

   public static final int BACK_RIGHT_DRIVE_ID = 19; // 1
   public static final int BACK_RIGHT_STEER_ID = 17; // 3
   public static final int BACK_RIGHT_TURN_ENCODER_ID = 2; // 0
   public static final double BACK_RIGHT_OFFSET = Math.toRadians(64.3993);
   ; // Math.toRadians(-46 + 180)
*/
