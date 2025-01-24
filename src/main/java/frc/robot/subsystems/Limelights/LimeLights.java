package frc.robot.subsystems.Limelights;

import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public class LimeLights {
    private ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter; // NICE IDEA IMPLEMENT LATER
    private String mLimeLight1;
    private Drive drivetrain;

    private Object[] lastMeasurement = null;

    private boolean validMeasurment = false;
    private double measurementTimeStamp;
    private Pose2d measurementPosition;

    public LimeLights(Drive drivetrain, String limeLightName) {
        this.drivetrain = drivetrain;
        this.mLimeLight1 = limeLightName;
    }

    public void periodic() {
        boolean doReject = false;
        // Use MegaTag2 because better?

        LimelightHelpers.SetRobotOrientation(
                "limelight-top", drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate limelightLeftMeasurment =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(mLimeLight1);
        if (limelightLeftMeasurment == null) {
            doReject = true;
        } else if (drivetrain.rotationRate
                < 4.0 * Math.PI) { // Only trust the measurment within a reasonable rotation rate
            doReject = true;
        } else if (limelightLeftMeasurment.tagCount == 0) {
            doReject = true;
        }
        validMeasurment = !doReject;
        if (validMeasurment) {
            System.out.println("READING-------------");
            measurementTimeStamp = limelightLeftMeasurment.timestampSeconds;
            measurementPosition = limelightLeftMeasurment.pose;
        }
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
