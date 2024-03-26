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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.vision.Vision;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SubsystemControl {
  private static final double DEADBAND = 0.1;

  private SubsystemControl() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude = Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = omegaSupplier.getAsDouble();

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getRotation()));
        },
        drive);
  }

  /*
   * Field oriented direction
   */
  public static Command fieldOrientedRotation(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier targetDirection,
      DoubleSupplier rotationLeft,
      DoubleSupplier rotationRight) {
    return Commands.run(
        () -> {
          Translation2d translation =
              new Translation2d(
                  xSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
                  ySupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec());

          double averageManualRotation = rotationLeft.getAsDouble() + rotationRight.getAsDouble();

          if (averageManualRotation != 0.0) {
            drive.setTargetDirection(drive.getRotation().getDegrees() + averageManualRotation);
          }

          if (targetDirection.getAsDouble() != -1.0) {
            drive.setTargetDirection(targetDirection.getAsDouble());
          }

          // Convert to field relative speeds & send command
          drive.runFieldOrientedDirection(translation);
        },
        drive);
  }

  public static Command limelightDrive(
      Drive drive,
      Vision vision,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          if (omega != 0.0d) { // Check if the driver isnt trying to turn
            vision.resetError();
          } else if ((omega == 0.0) && (vision.seesTarget())) {
            // Get tX from the vision subsystem. tX is "demand"
            omega = -vision.getDesiredAngle();
          }

          drive.runVelocity(
              // Convert to field relative speeds & send command
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getRotation()));
        },
        drive);
  }

  public static Command intakeWithColorSensor(
      Intake intake,
      Launcher launcher,
      DoubleSupplier reverseIntake,
      DoubleSupplier forwardIntake,
      BooleanSupplier runLauncher) {
    return Commands.run(
        () -> {
          // manual control
          if (reverseIntake.getAsDouble() > 0.1) {
            intake.setVelocity(-10.0 * reverseIntake.getAsDouble());
            launcher.setVelocity(-5.0 * reverseIntake.getAsDouble());
            return;
          }

          if (forwardIntake.getAsDouble() > 0.1) {
            intake.setVelocity(10.0 * forwardIntake.getAsDouble());
            launcher.setVelocity(5.0 * forwardIntake.getAsDouble());
            return;
          }

          // If the color sensor senses a note, disable the intake
          if (launcher.isNoteDetected()) {
            if (launcher.detectedNoteForSeconds() < 0.3) {
              intake.setVelocity(-0.5);
            } else {
              intake.disableIntake();
              if (runLauncher.getAsBoolean()) {
                launcher.enableLauncher();
              } else {
                launcher.disableLauncher();
              }
            }
          } else {
            intake.enableIntake();
            launcher.disableLauncher();
          }
        },
        intake,
        launcher);
  }
}
