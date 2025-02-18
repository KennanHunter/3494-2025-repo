package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public final class OI {
  private static EventLoop eventLoop = new EventLoop();
  public static XboxController primaryController =
      new XboxController(Constants.OI.PRIMARY_CONTROLLER_PORT);
  public static Joystick rightButtonBoard = new Joystick(2);

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
    
    double Sean_intake_power = deadband(-primaryController.getRightTriggerAxis() , Constants.Intake.DEADBAND) + deadband(primaryController.getLeftTriggerAxis(), Constants.Intake.DEADBAND);
    double Ashton_intake_power = deadband(rightButtonBoard.getRawAxis(0), Constants.Intake.DEADBAND);
    return   Sean_intake_power + Ashton_intake_power;
  }

  public static double getElevatorPower() {
    // TODO: assign a button
    double upPower = (primaryController.povUp(eventLoop).getAsBoolean()? -0.5: 0.0);
    double downPower = (primaryController.povDown(eventLoop).getAsBoolean()? 0.5 : 0.0);
    return upPower+downPower;
  }

  public static double getArmPower() {
    double leftPower = (primaryController.povLeft(eventLoop).getAsBoolean()? 1: 0.0);
    double rightPower = (primaryController.povRight(eventLoop).getAsBoolean()? -1 : 0.0);
    return leftPower+rightPower;
  }
  public static double getClimberPower(){
    double upPower = (primaryController.povUp(eventLoop).getAsBoolean()? 0.7: 0.0);
    double downPower = (primaryController.povDown(eventLoop).getAsBoolean()? -0.25 : 0.0);
    return upPower+downPower;
  }

}
