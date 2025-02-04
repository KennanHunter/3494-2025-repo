package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;

public final class OI {
  private static EventLoop eventLoop = new EventLoop();
  private static XboxController primaryController =
      new XboxController(Constants.OI.PRIMARY_CONTROLLER_PORT);

  public static XboxController getPrimaryController() {
    return primaryController;
  }

  public static void update() {
    eventLoop.poll();
  }

  public static double deadband(double input, double deadband) {
    if (Math.abs(input) > deadband) {
      if (input > 0.0) {
        return (input - deadband) / (1.0 - deadband);
      }

      return (input + deadband) / (1.0 - deadband);
    }

    return 0.0;
  }

  public static double getIntakePower() {
    return deadband(primaryController.getRightTriggerAxis() , Constants.Intake.DEADBAND) - deadband(primaryController.getLeftTriggerAxis(), Constants.Intake.DEADBAND);
  }

  public static double getElevatorPower() {
    // TODO: assign a button
    return 0.0;
  }

  public static double getArmPower() {
    // TODO: assign a button
    return 0.0;
  }
}
