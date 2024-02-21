package frc.robot.subsystems.ampscorer;

import org.littletonrobotics.junction.AutoLog;

public interface AmpScorerIO {
  @AutoLog
  public static class AmpScoreIOInputs {}

  public default void updateInputs(AmpScoreIOInputs inputs) {}
}
