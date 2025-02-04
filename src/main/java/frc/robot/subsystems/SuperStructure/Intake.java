package frc.robot.subsystems.SuperStructure;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkMax intakeMotor;
  private SparkMaxConfig intakeConfig;

  public Intake() {
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeMotor = new SparkMax(Constants.Intake.intakeMotorPort, MotorType.kBrushless);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }
}
