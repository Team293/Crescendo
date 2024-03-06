package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.Launcher;

public class Led extends SubsystemBase {
  private int kPort;
  private Spark m_led;
  private Launcher m_launcher;

  public Led(int port, Launcher launcher) {
    kPort = port;
    m_led = new Spark(kPort);
    m_launcher = launcher;
  }

  @Override
  public void periodic() {
    /* https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf */
    /* Page 14 for LED table */
    if (m_launcher.isNoteDetected()) {
      m_led.set(0.65); // orange
    } else {
      m_led.set(0.93); // Sky blue
    }
  }
}
