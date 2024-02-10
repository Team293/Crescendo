package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public boolean seesTarget = false;
    public double aprilTagId = 0;
    public double tX = 0.0;
    public double tY = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
