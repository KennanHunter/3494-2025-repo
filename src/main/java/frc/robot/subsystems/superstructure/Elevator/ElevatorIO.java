package frc.robot.subsystems.superstructure.Elevator;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public Distance currentHeight = Meters.of(0.0);
    public ElevatorSensorState sensorState = ElevatorSensorState.UP;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Sets the elevator position setpoint. */
  public default void runElevatorHeight(Distance height) {}

  /** Sets the brake mode. */
  public default void setBrakes(IdleMode idleMode) {}

  /** Resets the encoder position to given height. */
  public default void resetHeight(Distance height) {}

  /** Sets the PID output limits. */
  public default void setPIDlimits(double lowerBound, double upperBound) {}
}
