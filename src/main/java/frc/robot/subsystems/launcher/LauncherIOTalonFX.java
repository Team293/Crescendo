package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class LauncherIOTalonFX implements LauncherIO {
  public final TalonFX motor;

  private final StatusSignal<Double> motorVelocity;
  private final StatusSignal<Double> motorAppliedVolts;
  private final StatusSignal<Double> motorCurrent;

  private double setPoint = 0.0d;
  private double setPointError = 0.0d;
  private final double gearRatio = (11.0d / 10.0d);

  public LauncherIOTalonFX(int canId) {
    this.motor = new TalonFX(canId);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 0.85;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.18;
    config.Slot0.kS = 0.1d;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 100.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(config);

    motorVelocity = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50, motorVelocity, motorAppliedVolts, motorCurrent);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.motorVelocityRotationsPerSec = motorVelocity.getValueAsDouble();
    inputs.mechanismVelocityRotationsPerSec = inputs.motorVelocityRotationsPerSec * gearRatio;
    inputs.motorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.motorCurrentAmps = motorCurrent.getValueAsDouble();
    inputs.setPoint = setPoint;
    inputs.setPointError = motor.getClosedLoopError().getValueAsDouble();
  }

  @Override
  public void setVelocityRPS(double velocityRPS) {
    // the actual velocity is 125% over target
    setPoint = velocityRPS * gearRatio * 0.8;

    if (setPoint == 0) {
      motor.setControl(new VoltageOut(0));
    } else {
      motor.setControl(new VelocityVoltage(setPoint).withSlot(0));
    }
  }
}
