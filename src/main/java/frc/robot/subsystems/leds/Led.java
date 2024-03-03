package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.ColorSensorIOInputsAutoLogged;

public class Led extends SubsystemBase {
  private int kPort;
  private Spark m_led;
  private ColorSensorIOInputsAutoLogged m_colorSensorInputs = new ColorSensorIOInputsAutoLogged();

  public Led(int port) {
    kPort = port;
    m_led = new Spark(kPort);
  }

  @Override
  public void periodic() {
    /* https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf */
    /* Page 14 for LED table */
    if (m_colorSensorInputs.IsNoteDetected) {
      m_led.set(0.57); // Hot pink
    } else {
      m_led.set(0.83); // Sky blue
    }
  }
}
