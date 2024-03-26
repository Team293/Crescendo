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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Module {
  private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
  private static final double WHEEL_CIRCUMFERENCE = 2.0 * WHEEL_RADIUS * Math.PI;
  public static final double ODOMETRY_FREQUENCY = 250.0;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  /* Velocity in meters per second
   *
   */
  private Double speedSetpoint = null;
  private double lastPositionMeters = 0.0; // Used for delta calculation
  private SwerveModulePosition[] positionDeltas = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    setBrakeMode(true);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    // Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    // if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
    //   turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
    // }

    // Run closed loop turn control
    if (angleSetpoint != null) {
      io.setTurnPosition(angleSetpoint);

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (speedSetpoint != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double angleError =
            Math.abs(rotationalDifferenceBetween(inputs.turnPosition, angleSetpoint));
        double adjustSpeedSetpoint = speedSetpoint * Math.cos(Units.degreesToRadians(angleError));

        // Run drive controller, convert meters per second to rotations per second
        double velocityRotationsPerSec = adjustSpeedSetpoint / WHEEL_CIRCUMFERENCE;

        io.setDriveVelocityRPS(velocityRotationsPerSec);
      }
    }

    // Calculate position deltas for odometry
    positionDeltas = new SwerveModulePosition[1];
    for (int i = 0; i < 1; i++) {
      double positionMeters = inputs.drivePositionRotations * WHEEL_CIRCUMFERENCE;
      // Rotation2d angle =
      //     inputs.turnPosition.plus(
      //         turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d());
      Rotation2d angle = inputs.turnPosition;
      positionDeltas[i] = new SwerveModulePosition(positionMeters - lastPositionMeters, angle);
      lastPositionMeters = positionMeters;
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.setDriveVoltage(volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /* Sets the brake mode for the drive motor only */
  public void setDriveBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    // if (turnRelativeOffset == null) {
    //   return new Rotation2d();
    // } else {
    return inputs.turnPosition; // .plus(turnRelativeOffset);
    // }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRotations * WHEEL_CIRCUMFERENCE;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRotationsPerSec * WHEEL_CIRCUMFERENCE;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position deltas received this cycle. */
  public SwerveModulePosition[] getPositionDeltas() {
    return positionDeltas;
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRotationsPerSec;
  }

  /** Returns the difference between rotations in degrees. */
  public double rotationalDifferenceBetween(Rotation2d a, Rotation2d b) {
    double aDeg = a.getDegrees();
    double bDeg = b.getDegrees();

    double diff = Math.abs(aDeg - bDeg) % 360.0;
    if (diff > 180.0) {
      diff = 360.0 - diff;
    }

    return diff;
  }
}
