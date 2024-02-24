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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriverCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.OperatorCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Launcher launcher;
  private final Vision vision;
  private final Intake intake;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> autoChooser2;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    /* Print the log directory */
    String logDir = DataLogManager.getLogDir();
    System.out.print(logDir);

    // Initialize the intake subsystem

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIONavX(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        break;
    }
    vision = new Vision();
    launcher = new Launcher();

    // Initalize intake
    intake = new Intake(drive);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser2 = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser2", autoChooser2);

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Drive command */
      // drive.setDefaultCommand(
      //     DriverCommands.joystickDrive(
      //         drive,
      //         () -> -driverController.getRightY(),
      //         () -> -driverController.getRightX(),
      //         () -> -driverController.getLeftX()));
      
      drive.setDefaultCommand(
          DriverCommands.limelightDrive(
              drive,
              vision,
              () -> -driverController.getRightY(),
              () -> -driverController.getRightX(),
              () -> -driverController.getLeftX()));

    /* Brake command */
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    /* Reset heading command */
    driverController
        .b()
        .onTrue(Commands.runOnce(drive::resetRotation, drive).ignoringDisable(true));

    /* Intake command */
    SequentialCommandGroup enableIntake = new SequentialCommandGroup(
        Commands.waitSeconds(1.000), Commands.runOnce(intake::enableIntake));

    ParallelCommandGroup enableLauncher = new ParallelCommandGroup(Commands.runOnce(launcher::enableLauncher),
        enableIntake);

    ParallelCommandGroup disableLauncher = new ParallelCommandGroup(
        Commands.runOnce(launcher::disableLauncher),
        Commands.runOnce(intake::disableIntake));

    intake.setDefaultCommand(
        OperatorCommands.defaultOperator(intake, operatorController::getLeftY));

    // operatorController.leftBumper().whileTrue(Commands.runOnce(intake::enableIntake,
    // intake));
    // operatorController.leftBumper().whileFalse(Commands.runOnce(intake::disableIntake,
    // intake));

    operatorController.rightBumper().whileTrue(enableLauncher);
    operatorController.rightBumper().whileFalse(disableLauncher);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser2.getSelected();
  }
}
