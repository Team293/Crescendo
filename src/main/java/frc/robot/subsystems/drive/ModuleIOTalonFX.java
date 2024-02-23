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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.constants.SDSMK4L1Constants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = SDSMK4L1Constants.driveGearRatio;
  private static final double TURN_GEAR_RATIO = SDSMK4L1Constants.angleGearRatio;

  private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
  private static final double WHEEL_CIRCUMFERENCE = 2.0 * WHEEL_RADIUS * Math.PI;

  private final double absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(0);
        turnTalon = new TalonFX(1);
        cancoder = new CANcoder(2);
        absoluteEncoderOffset = 0.121d; // CANCoder rotations
        break;
      case 1:
        driveTalon = new TalonFX(3);
        turnTalon = new TalonFX(4);
        cancoder = new CANcoder(5);
        absoluteEncoderOffset = -0.29d; // CANcoder rotations
        break;
      case 2:
        driveTalon = new TalonFX(6);
        turnTalon = new TalonFX(7);
        cancoder = new CANcoder(8);
        absoluteEncoderOffset = 0.134d; // CANcoder rotations
        break;
      case 3:
        driveTalon = new TalonFX(9);
        turnTalon = new TalonFX(10);
        cancoder = new CANcoder(11);
        absoluteEncoderOffset = 0.227d; // CANcoder rotations
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    /* Drive motor config */
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted = SDSMK4L1Constants.driveMotorInvert;

    driveConfig.Slot0.kP = SDSMK4L1Constants.driveKP;
    driveConfig.Slot0.kI = SDSMK4L1Constants.driveKI;
    driveConfig.Slot0.kD = SDSMK4L1Constants.driveKD;

    driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

    driveTalon.clearStickyFaults();
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    /* Turn motor config */
    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.MotorOutput.Inverted = SDSMK4L1Constants.angleMotorInvert;

    turnConfig.Slot0.kP = SDSMK4L1Constants.angleKP;
    turnConfig.Slot0.kI = SDSMK4L1Constants.angleKI;
    turnConfig.Slot0.kD = SDSMK4L1Constants.angleKD;

    turnConfig.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;

    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    // CANcoder config
    var canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SDSMK4L1Constants.canCoderSensorDirection;
    canCoderConfig.MagnetSensor.MagnetOffset = -absoluteEncoderOffset;
    cancoder.getConfigurator().apply(canCoderConfig);

    // Drive motor config
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Turn motor
    turnAbsolutePosition = cancoder.getAbsolutePosition(); /* -0.5 <-> 0.5 in mechanism rotations */
    /* Mechanism rotations -> motor rotations */
    turnTalon.setPosition(turnAbsolutePosition.getValueAsDouble());
    turnPosition = turnTalon.getPosition(); // motor rotations
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.canCoderRotations = cancoder.getAbsolutePosition().getValue();
    inputs.canCoderAngle = Units.rotationsToDegrees(inputs.canCoderRotations);

    inputs.drivePositionRotations = drivePosition.getValueAsDouble();
    inputs.driveVelocityRotationsPerSec = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnPositionAngle = inputs.turnPosition.getDegrees();
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    inputs.odometryDrivePositionsRotations =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));

    driveTalon.setControl(new VelocityVoltage(volts).withSlot(0));
  }

  @Override
  public void setDriveVelocityRPS(double velocityRPS) {
    driveTalon.setControl(new VelocityVoltage(velocityRPS).withSlot(0));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = SDSMK4L1Constants.driveMotorInvert;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = SDSMK4L1Constants.angleMotorInvert;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}
