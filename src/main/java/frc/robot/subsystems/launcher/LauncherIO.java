package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    public double motorVelocityRadPerSec = 0.0d;
    public double motorAppliedVolts = 0.0d;
    public double motorCurrentAmps = 0.0d;
    public double motorTargetRPS = 0.0d;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public default void setSpeed(double speed) {}
}
