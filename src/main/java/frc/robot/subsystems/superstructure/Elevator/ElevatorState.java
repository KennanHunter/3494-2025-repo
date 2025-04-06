package frc.robot.subsystems.superstructure.Elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public record ElevatorState(Distance height, LinearVelocity velocity, IdleMode idleMode) {}
