package frc.robot.subsystems.limelights;

import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public class Limelights {
  private ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter; // NICE IDEA IMPLEMENT LATER
  private String mLimeLight1;
  private Drive drivetrain;

  private Object[] lastMeasurement = null;

  private boolean validMeasurment = false;
  private double measurementTimeStamp;
  private Pose2d measurementPosition;

  public Limelights(Drive drivetrain, String limeLightName) {
    this.drivetrain = drivetrain;
    this.mLimeLight1 = limeLightName;
  }

  public void periodic() {
    // Use MegaTag2 because better?
    LimelightHelpers.SetRobotOrientation(
        "limelight-top", drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate limelightLeftMeasurment =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(mLimeLight1);

    boolean leftLimelightEmpty = limelightLeftMeasurment == null;
    boolean rotationRateTooHigh = drivetrain.rotationRate < 4.0 * Math.PI;
    boolean noTagsFound = limelightLeftMeasurment.tagCount == 0;

    if (leftLimelightEmpty || rotationRateTooHigh || noTagsFound) {
      validMeasurment = false;
      return;
    }

    validMeasurment = true;
    System.out.println("READING-------------");
    measurementTimeStamp = limelightLeftMeasurment.timestampSeconds;
    measurementPosition = limelightLeftMeasurment.pose;
  }

  // returns validity of last measurment
  public boolean measurmentValid() {
    return validMeasurment;
  }

  public double getMeasurementTimeStamp() {
    return measurementTimeStamp;
  }

  public Pose2d getMeasuremPosition() {
    return measurementPosition;
  }
}
