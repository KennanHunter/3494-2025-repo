package frc.robot.subsystems.superstructure;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private SparkMax intakeMotor;
  private SparkMaxConfig intakeConfig;
  private double intakeSpeed;

  boolean hasNote = false;
  private double currentSpeed = 0;
  ArrayList currents = new ArrayList<>();

  public Intake() {
    Logger.recordOutput("Intake/Intake-Power", intakeSpeed);
    intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.smartCurrentLimit(40);
    intakeConfig.inverted(false);
    intakeMotor = new SparkMax(Constants.Intake.intakeMotor, MotorType.kBrushless);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    currentSpeed = speed;
    intakeMotor.set(speed);
  }

  public double currentAverage(double currentCurrent) {
    currents.add(0, currentCurrent);

    if (currents.size() >= 20) {
      currents.remove(currents.size() - 1);
    }
    double average = 0;
    for (int i = 0; i < currents.size(); i++) {
      average += (double) currents.get(i);
    }
    average /= currents.size();
    return average;
  }

  @Override
  public void periodic() {
    double Cavrg = currentAverage(intakeMotor.getOutputCurrent());
    if (Cavrg > 9) {
      hasNote = true;
    }
    Logger.recordOutput("Intake/Current-Average", Cavrg);
    Logger.recordOutput("Intake/Has-Coral", hasNote);
    if (currentSpeed > 0.05) {
      hasNote = false;
    }
  }

  public boolean hasNote() {
    return hasNote;
  }
}
