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
  private double setpoint = 0.0d;
  private final double m_gearRatio = (4.0 / 1.0); // 4 motor rotations per 1 intake rotation

  private static VelocityVoltage velocityVoltageCommand = new VelocityVoltage(0.0).withSlot(0);

  public IntakeIOTalonFX(int canId) {
    this.motor = new TalonFX(canId, "rio");
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = m_gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set motor PID
    config.Slot0.kP = 0.11;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.12;
    config.Slot0.kS = 0.05;
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

    inputs.motorVelocityRotationsPerSecond = motorVelocity.getValueAsDouble();
    inputs.motorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.motorCurrentAmps = motorCurrent.getValueAsDouble();
    // inputs.robotSpeed = this.robotSpeed;
    inputs.setPoint = setpoint;
    inputs.setPointError = motor.getClosedLoopError().getValueAsDouble();
  }

  // Takes in speed as pulley rotations per second
  @Override
  public void setSpeed(double speed) {
    velocityVoltageCommand.withVelocity(
        speed * m_gearRatio); // Convert to motor rotations per second
    motor.setControl(velocityVoltageCommand);
  }
}
