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

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

/**
 * This drive implementation is for Talon FXs driving brushless motors like the Falon 500 or Kraken
 * X60.
 */
public class IntakeIOTalonFX implements IntakeIO {
  public final TalonFX motor;
  private double robotSpeed; // Robot speed from the drivetrain

  private final StatusSignal<Double> motorVelocity;
  private final StatusSignal<Double> motorAppliedVolts;
  private final StatusSignal<Double> motorCurrent;

  public IntakeIOTalonFX(int canId) {
    this.motor = new TalonFX(canId);
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set motor PID
    config.Slot0.kP = 1.0; // TODO: config
    config.Slot0.kI = 0.0; // TODO: config
    config.Slot0.kD = 0.0; // TODO: config
    config.Slot0.kV = (5.0 / 12.0); // RPS per volt
    config.Slot0.kA = (0.096 / 12.0);
    motor.getConfigurator().apply(config);

    motorVelocity = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVelocity, motorAppliedVolts, motorCurrent);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs, double robotSpeed) {
    this.robotSpeed = robotSpeed;
    BaseStatusSignal.refreshAll(motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.motorVelocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.motorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.motorCurrentAmps = motorCurrent.getValueAsDouble();
    inputs.robotSpeed = this.robotSpeed;
  }

  @Override
  public void setSpeed(double speed) {
    double inputSpeed = -(speed + this.robotSpeed);
    motor.setControl(new VelocityVoltage(inputSpeed).withSlot(0));
  }
}