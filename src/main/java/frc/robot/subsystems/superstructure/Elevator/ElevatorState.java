package frc.robot.subsystems.superstructure.Elevator;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Distance;

public record ElevatorState(Distance height, IdleMode idleMode) {
  // TODO: I wrote this while talking to someone in my head that was insulting my code, does it make
  // sense?
  static ElevatorState holdAtMeters(double val) {
    return new ElevatorState(Meters.of(val), IdleMode.kBrake);
  }
}
