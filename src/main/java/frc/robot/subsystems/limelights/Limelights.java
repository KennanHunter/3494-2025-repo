package frc.robot.subsystems.limelights;

import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class Limelights {
  private ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter; // NICE IDEA IMPLEMENT LATER

  private boolean invalidMeasurment = true;
  private double measurementTimeStamp;
  private Pose2d measurementPosition;

  private LimelightIOInputsAutoLogged inputs;
  private LimelightIO limelightIO;

  public Limelights(Drive drivetrain, String limeLightName) {
    inputs = new LimelightIOInputsAutoLogged();
  }

  public void periodic() {
    limelightIO.updateInputs(inputs);
    Logger.processInputs("Limelights", inputs);

    LimelightHelpers.PoseEstimate measurement = inputs.limelightMeasurement;

    if (measurement == null || inputs.isDrivetrainRotationRateTooHigh) {
      invalidMeasurment = true;
      return;
    }

    invalidMeasurment = false;

    measurementTimeStamp = measurement.timestampSeconds();
    measurementPosition = measurement.pose();
  }

  // returns validity of last measurment
  public boolean measurmentValid() {
    return !invalidMeasurment;
  }

  public double getMeasurementTimeStamp() {
    return measurementTimeStamp;
  }

  public Pose2d getMeasuremPosition() {
    return measurementPosition;
  }
}
