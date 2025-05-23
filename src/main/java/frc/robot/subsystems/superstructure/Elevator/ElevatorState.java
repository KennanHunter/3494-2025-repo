package frc.robot.subsystems.superstructure.Elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Distance;

public record ElevatorState(Distance height, IdleMode idleMode) {}
