package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class SeanMathUtil {
  public static double distance(Pose2d pos1, Pose2d pos2) {
    // Can also be written as
    // return pos1.getTranslation().getDistance(pos2.getTranslation());

    double xdist = pos2.getX() - pos1.getX();
    double ydist = pos2.getY() - pos1.getY();
    return Math.sqrt(Math.pow(ydist, 2) + Math.pow(xdist, 2));
  }
}
