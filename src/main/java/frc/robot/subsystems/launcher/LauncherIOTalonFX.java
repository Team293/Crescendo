package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LauncherIOTalonFX implements LauncherIO {
  public final TalonFX motor;

  private final StatusSignal<Double> motorVelocity;
  private final StatusSignal<Double> motorAppliedVolts;
  private final StatusSignal<Double> motorCurrent;

  private double targetSpeed;
  private final double gearRation = (11 / 10);

  public LauncherIOTalonFX(int canId) {
    this.motor = new TalonFX(canId);
    var config = new TalonFXConfiguration();

    config.Slot0.kP = 1.0; // TODO: config
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = (50.0 / 12.0); // Max RPMs per volt
    config.Slot0.kA = (0.096 / 12.0);
    motor.getConfigurator().apply(config);

    targetSpeed = SmartDashboard.getNumber("launcher speed(RPS)", 0); // rps

    motorVelocity = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50, motorVelocity, motorAppliedVolts, motorCurrent);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorVelocity, motorAppliedVolts, motorCurrent);

    targetSpeed = SmartDashboard.getNumber("launcher speed(RPS)", 0); // rps

    inputs.motorVelocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.motorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.motorCurrentAmps = motorCurrent.getValueAsDouble();
    inputs.motorTargetRPS = targetSpeed;
  }

  @Override
  public void setSpeed(double speed) {
    targetSpeed = speed;
    motor.setControl(new VelocityVoltage(targetSpeed * gearRation).withSlot(0));
  }
}
