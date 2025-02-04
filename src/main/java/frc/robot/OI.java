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

  public static double deadband(double input) {
    if (Math.abs(input) > Constants.OI.DEADBAND) {
      if (input > 0.0) {
        return (input - Constants.OI.DEADBAND) / (1.0 - Constants.OI.DEADBAND);
      }

      return (input + Constants.OI.DEADBAND) / (1.0 - Constants.OI.DEADBAND);
    }

    return 0.0;
  }

  public static double intakeIn() {
    return deadband(primaryController.getRightTriggerAxis());
  }

  public static double intakeOut() {
    return deadband(primaryController.getLeftTriggerAxis());
  }
}
