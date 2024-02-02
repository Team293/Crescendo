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
  private final TalonFX launchMotor = new TalonFX(10); // TODO: change this to the correct ID
  private final TalonFX feedMotor = new TalonFX(11); // TODO: change this to the correct ID

  private final StatusSignal<Double> launchVelocity = launchMotor.getVelocity();
  private final StatusSignal<Double> launchAppliedVolts = launchMotor.getMotorVoltage();
  private final StatusSignal<Double> launchCurrent = launchMotor.getStatorCurrent();

  private final StatusSignal<Double> feedVelocity = feedMotor.getVelocity();
  private final StatusSignal<Double> feedAppliedVolts = feedMotor.getMotorVoltage();
  private final StatusSignal<Double> feedCurrent = feedMotor.getStatorCurrent();

  public IntakeIOTalonFX() { 
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    launchMotor.getConfigurator().apply(config);
    feedMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        launchVelocity,
        launchAppliedVolts,
        launchCurrent,
        feedVelocity,
        feedAppliedVolts,
        feedCurrent);
    launchMotor.optimizeBusUtilization();
    feedMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        launchVelocity,
        launchAppliedVolts,
        launchCurrent,
        feedVelocity,
        feedAppliedVolts,
        feedCurrent);

    inputs.launchVelocityRadPerSec = Units.rotationsToRadians(launchVelocity.getValueAsDouble());
    inputs.launchAppliedVolts = launchAppliedVolts.getValueAsDouble();
    inputs.launchCurrentAmps = launchCurrent.getValueAsDouble();

    inputs.feedVelocityRadPerSec = Units.rotationsToRadians(feedVelocity.getValueAsDouble());
    inputs.feedAppliedVolts = feedAppliedVolts.getValueAsDouble();
    inputs.feedCurrentAmps = feedCurrent.getValueAsDouble();
  }

  @Override
  public void setSpeed(double speed) {
    launchMotor.setControl(new VelocityVoltage(speed));
  }
}