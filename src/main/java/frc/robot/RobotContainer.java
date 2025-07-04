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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.JoyStickDriveCommand;
import frc.robot.commands.superstructure.ReturnSuperStructureToSafeState;
import frc.robot.commands.superstructure.SuperStructureManualControlCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.superstructure.Arm.ArmIO;
import frc.robot.subsystems.superstructure.Arm.ArmIOSim;
import frc.robot.subsystems.superstructure.Arm.ArmIOSpark;
import frc.robot.subsystems.superstructure.Elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.Elevator.ElevatorIOReal;
import frc.robot.subsystems.superstructure.Elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.KnownState;
import frc.robot.subsystems.superstructure.SuperStructure;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final SuperStructure superStructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  public static Joystick leftButtonBoard = new Joystick(1);
  public static Joystick rightButtonBoard = new Joystick(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Logger.recordMetadata("Current Mode", Constants.currentMode.toString());

    switch (Constants.currentMode) {
      /* Real robot, instantiate hardware IO implementations */
      case REAL -> {
        superStructure = new SuperStructure(new ElevatorIOReal(), new ArmIOSpark());

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
      }

      /* Sim robot, instantiate physics sim IO implementations */
      case SIM -> {
        superStructure = new SuperStructure(new ElevatorIOSim(), new ArmIOSim());

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
      }

      /* Replayed robot, disable IO implementations */
      default -> {
        superStructure = new SuperStructure(new ElevatorIO() {}, new ArmIO() {});

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
      }
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        new JoyStickDriveCommand(
            drive,
            () -> -controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .a()
        .onTrue(
            new SuperStructureManualControlCommand(
                superStructure, () -> controller.getLeftX(), () -> controller.getLeftY()));

    controller.y().onTrue(new ReturnSuperStructureToSafeState(superStructure));

    controller.x().onTrue(superStructure.createCommandTraversalToKnownState(KnownState.Test2));

    controller.b().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .back()
        .onTrue(
            Commands.runOnce(
                () -> {
                  GyroIOPigeon2.pigeon.setYaw(0.0);
                }));

    controller
        .start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.rezeroModulesRelativeEncoders();
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
