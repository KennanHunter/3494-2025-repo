package frc.robot.subsystems.SuperStructure.Arm;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {
  DCMotor armGearbox = DCMotor.getNeoVortex(1);

  SparkFlex armMotor = new SparkFlex(Constants.Arm.ARM_MOTOR_CAN_ID, MotorType.kBrushless);

  SparkFlexSim armMotorSim = new SparkFlexSim(armMotor, armGearbox);

  private void stepSimulation() {
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(armMotorSim.getVelocity()),
        RoboRioSim.getVInVoltage(),
        0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armMotorSim.getMotorCurrent()));

    armMotor.getEncoder().setPosition(Units.radiansToRotations(armMotorSim.getPosition()));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    stepSimulation();

    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public double getArmPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getArmPosition'");
  }

  @Override
  public void setTargetPosition(double targetPosition) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTargetPosition'");
  }

  @Override
  public void setBrakes(IdleMode neutralMode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setBrakes'");
  }
}
