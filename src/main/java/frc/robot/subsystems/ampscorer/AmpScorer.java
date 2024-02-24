package frc.robot.subsystems.ampscorer;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpScorer extends SubsystemBase {
  private final AmpScorerIO m_ampScorerIO;
  private final Spark m_motor;

  public AmpScorer() {
    m_ampScorerIO = new AmpScorerIOSpark();
    m_motor = new Spark(0);
  }

  public void intakeNote() {
    m_motor.set(-0.5);
  }

  public void dischargeNote() {
    m_motor.set(0.75);
  }

  public void stop() {
    m_motor.set(0);
  }
}
