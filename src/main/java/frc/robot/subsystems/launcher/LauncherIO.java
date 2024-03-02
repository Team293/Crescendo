package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    public double motorVelocityRotationsPerSec = 0.0d;
    public double mechanismVelocityRotationsPerSec = 0.0d;
    public double motorAppliedVolts = 0.0d;
    public double motorCurrentAmps = 0.0d;
    public double setPoint = 0.0d;
    public double setPointError = 0.0d;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public default void setVelocityRPS(double speed) {}
}
