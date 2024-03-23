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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.SpikeController;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.SubsystemControl;
import frc.robot.commands.note.ColorSensorIntake;
import frc.robot.commands.note.Launch;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.leds.Led;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Launcher launcher;
  // private final Vision vision;
  private final Intake intake;
  private final Led led;

  // Controller
  private static final double DEADBAND = 0.05;
  private final SpikeController driverController = new SpikeController(0, DEADBAND);
  private final SpikeController operatorController = new SpikeController(1, DEADBAND);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /* Print the log directory */
    String logDir = DataLogManager.getLogDir();
    System.out.print(logDir);

    // Initialize the intake subsystem

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
    // Initalize subsystems
    // vision = new Vision();
    launcher = new Launcher();
    intake = new Intake(drive);
    led = new Led(1, launcher);

    NamedCommands.registerCommand("launchNote", new Launch(intake, launcher));
    NamedCommands.registerCommand("colorSensorIntake", new ColorSensorIntake(intake, launcher));
    NamedCommands.registerCommand("launchNote2", new Launch(intake, launcher));
    NamedCommands.registerCommand("colorSensorIntake2", new ColorSensorIntake(intake, launcher));
    NamedCommands.registerCommand("launchNote3", new Launch(intake, launcher));
    NamedCommands.registerCommand("colorSensorIntake3", new ColorSensorIntake(intake, launcher));
    NamedCommands.registerCommand("launchNote4", new Launch(intake, launcher));
    NamedCommands.registerCommand("colorSensorIntake", new ColorSensorIntake(intake, launcher));

    // Initalize climber

    // Set up auto routines
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

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
    /* Drive command */
    drive.setDefaultCommand(
        SubsystemControl.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    /*SubsystemControl.fieldOrientedRotation(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> {
                  Rotation2d rot =
                      new Rotation2d(driverController.getRightX(), driverController.getRightY());
                  double magnitude =
                      Math.hypot(driverController.getRightX(), driverController.getRightY());

                  if (magnitude > 0.5) {
                    return (-rot.getDegrees() + 90) % 360;
                  } else {
                    return -1;
                  }
                },
                () -> driverController.getLeftTriggerAxis(),
                () -> driverController.getRightTriggerAxis()));
    */
    /* Brake command */
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    /* Reset heading command */
    driverController
        .y()
        .onTrue(Commands.runOnce(() -> drive.resetRotation(0.0), drive).ignoringDisable(true));
    ;

    /* Reset heading command */
    driverController
        .a()
        .onTrue(Commands.runOnce(() -> drive.resetRotation(180.0), drive).ignoringDisable(true));

    /* Intake auto-run command */
    /* Reverse intake control as well */
    intake.setDefaultCommand(
        SubsystemControl.intakeWithColorSensor(
            intake,
            launcher,
            operatorController::getLeftTriggerAxis,
            operatorController::getRightTriggerAxis,
            () -> operatorController.leftBumper().getAsBoolean()));

    /* Launcher control */
    operatorController
        .rightBumper()
        .onTrue(Commands.runOnce(() -> new Launch(intake, launcher).schedule(), intake, launcher));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
