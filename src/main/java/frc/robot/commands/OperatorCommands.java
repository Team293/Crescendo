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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.ColorSensorIORevV3;
import frc.robot.subsystems.launcher.Launcher;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class OperatorCommands {
  private static final double DEADBAND = 0.1;
  private static final double MAX_INTAKE_SPEED_RPS = 20.0;

  private OperatorCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command defaultOperator(
    Intake intake,
    Launcher launcher,
    ColorSensorIORevV3 colorSensor,
    DoubleSupplier intakeSpeed,
    BooleanSupplier launchButton) {
    return Commands.run(
        () -> {
          double deadbandedSpeed = MathUtil.applyDeadband(intakeSpeed.getAsDouble(), DEADBAND);
          deadbandedSpeed = MathUtil.clamp(deadbandedSpeed, -1.0, 1.0);

          if (launchButton.getAsBoolean()) {
            // set the velocity of the launcher
            // reverse intake speed until the note isnt visible
            // wait for the launcher to spin up to speed
            // run the intake forward
            // wait for a certain amount of time
            // turn off both
            return;
          }

          intake.setVelocity(deadbandedSpeed * MAX_INTAKE_SPEED_RPS);
        },
        intake);
  }
}
