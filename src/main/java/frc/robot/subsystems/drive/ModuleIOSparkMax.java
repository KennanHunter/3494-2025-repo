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

package frc.robot.subsystems.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import java.util.OptionalDouble;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final SparkFlex driveSparkFlex;
  private final SparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true; // FIX MEEE
  private final Rotation2d absoluteEncoderOffset;

  private Rotation2d lastRawTurnEncoderPosition;

  private SparkMaxConfig driveSparkMaxConfig = new SparkMaxConfig();
  private SparkMaxConfig turnSparkMaxConfig = new SparkMaxConfig();

  private int index;

  public ModuleIOSparkMax(int index) {
    this.index = index;

    switch (index) {
      case 0:
        driveSparkFlex =
            new SparkFlex(Constants.Drivetrain.FRONT_LEFT_DRIVE_ID, MotorType.kBrushless);
        turnSparkMax = new SparkMax(Constants.Drivetrain.FRONT_LEFT_STEER_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(Constants.Drivetrain.FRONT_LEFT_TURN_ENCODER_ID);
        absoluteEncoderOffset =
            new Rotation2d(Constants.Drivetrain.FRONT_LEFT_OFFSET); // MUST BE CALIBRATED
        break;
      case 1:
        driveSparkFlex =
            new SparkFlex(Constants.Drivetrain.FRONT_RIGHT_DRIVE_ID, MotorType.kBrushless);
        turnSparkMax =
            new SparkMax(Constants.Drivetrain.FRONT_RIGHT_STEER_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(Constants.Drivetrain.FRONT_RIGHT_TURN_ENCODER_ID);
        absoluteEncoderOffset =
            new Rotation2d(Constants.Drivetrain.FRONT_RIGHT_OFFSET); // MUST BE CALIBRATED
        break;
      case 2:
        driveSparkFlex =
            new SparkFlex(Constants.Drivetrain.BACK_LEFT_DRIVE_ID, MotorType.kBrushless);
        turnSparkMax = new SparkMax(Constants.Drivetrain.BACK_LEFT_STEER_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(Constants.Drivetrain.BACK_LEFT_TURN_ENCODER_ID);
        absoluteEncoderOffset =
            new Rotation2d(Constants.Drivetrain.BACK_LEFT_OFFSET); // MUST BE CALIBRATED
        break;
      case 3:
        driveSparkFlex =
            new SparkFlex(Constants.Drivetrain.BACK_RIGHT_DRIVE_ID, MotorType.kBrushless);
        turnSparkMax = new SparkMax(Constants.Drivetrain.BACK_RIGHT_STEER_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(Constants.Drivetrain.BACK_RIGHT_TURN_ENCODER_ID);
        absoluteEncoderOffset =
            new Rotation2d(Constants.Drivetrain.BACK_RIGHT_OFFSET); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }
    driveSparkMaxConfig.voltageCompensation(12);
    driveSparkMaxConfig.smartCurrentLimit(40);
    driveSparkMaxConfig.signals.primaryEncoderPositionPeriodMs(
        (int) (1000.0 / Module.ODOMETRY_FREQUENCY));

    turnSparkMaxConfig.smartCurrentLimit(30);
    turnSparkMaxConfig.voltageCompensation(12);
    turnSparkMaxConfig.signals.primaryEncoderPositionPeriodMs(
        (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkMaxConfig.inverted(isTurnMotorInverted);

    driveSparkFlex.configure(
        driveSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnSparkMax.configure(
        turnSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveSparkFlex.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkFlex.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    // TODO: Test the removal of these two lines
    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkFlex.setInverted(true);

    driveEncoder.setPosition(0.0);
    // driveEncoder.setMeasurementPeriod(10);
    // driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    // turnRelativeEncoder.setMeasurementPeriod(10);
    // turnRelativeEncoder.setAverageDepth(2);

    driveSparkFlex.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();

    drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = driveEncoder.getPosition();
                  if (driveSparkFlex.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = turnRelativeEncoder.getPosition();
                  if (turnSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    Logger.recordOutput("Odometry/DriveOutputQueueLength", drivePositionQueue.size());

    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkFlex.getAppliedOutput() * driveSparkFlex.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkFlex.getOutputCurrent()};

    inputs.rawTurnEncoderPosition =
        new Rotation2d(
            turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI);
    this.lastRawTurnEncoderPosition = inputs.rawTurnEncoderPosition;
    inputs.turnAbsolutePosition = inputs.rawTurnEncoderPosition.minus(absoluteEncoderOffset);

    Logger.recordOutput("Encoder" + index + " output voltage", turnAbsoluteEncoder.getVoltage());
    Logger.recordOutput(
        "Encoder" + index + " output rotation", inputs.turnAbsolutePosition.getDegrees());
    // Logger.recordOutput("Encoder" + index + " output rotation", inputs.)
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();

    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkFlex.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMaxConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    driveSparkFlex.configure(
        driveSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMaxConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    turnSparkMax.configure(
        turnSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public Rotation2d getRawTurnEncoderPosition() {
    return lastRawTurnEncoderPosition;
  }
}
