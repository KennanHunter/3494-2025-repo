package frc.robot.subsystems.superstructure.Intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private SparkMax intakeMotor;
  private Optional<IntakeState> state = Optional.empty();

  public Intake() {
    intakeMotor = new SparkMax(Constants.Intake.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.smartCurrentLimit(40);
    intakeConfig.inverted(false);

    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTargetState(IntakeState state) {
    this.state = Optional.of(state);
    Logger.recordOutput("Intake/State", state);

    // TODO: Constantize and make actually work
    double speed =
        switch (state) {
          case Hold -> -0.4;
          case Spit -> 0.6;
        };

    intakeMotor.set(speed);
  }

  public Optional<IntakeState> getState() {
    return this.state;
  }

  @Override
  public void periodic() {}

  public boolean isAtTarget() {
    // TODO: Intake state is so inconsequential rn that i just return true, should probably properly
    // implement at some point esp. for auto routines that may require spitting as a task
    return true;
  }
}
