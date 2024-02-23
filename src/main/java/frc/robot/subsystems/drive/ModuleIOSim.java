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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.constants.SDSMK4L1Constants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim driveSim =
      new DCMotorSim(DCMotor.getNEO(1), SDSMK4L1Constants.driveGearRatio, 0.025);
  private DCMotorSim turnSim =
      new DCMotorSim(DCMotor.getNEO(1), SDSMK4L1Constants.angleGearRatio, 0.004);

  private SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(SDSMK4L1Constants.motorKS, SDSMK4L1Constants.motorKV);
  private PIDController driveFeedback =
      new PIDController(
          SDSMK4L1Constants.driveKP, SDSMK4L1Constants.driveKI, SDSMK4L1Constants.driveKD);
  private PIDController turnFeedback =
      new PIDController(
          SDSMK4L1Constants.angleKP, SDSMK4L1Constants.angleKI, SDSMK4L1Constants.angleKD);

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private double currentVelocity = 0.0;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionRotations = driveSim.getAngularPositionRad();
    inputs.driveVelocityRotationsPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};

    inputs.odometryDrivePositionsRotations = new double[] {inputs.drivePositionRotations};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};

    // for simulated PID only
    currentVelocity = inputs.driveVelocityRotationsPerSec;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveVelocity(double targetVelocity) {
    // convert from rotations per second to radians per second
    double radPerSec = targetVelocity * 2 * Math.PI;
    setDriveVoltage(
        driveFeedforward.calculate(radPerSec)
            + driveFeedback.calculate(currentVelocity, radPerSec));
  }
}
