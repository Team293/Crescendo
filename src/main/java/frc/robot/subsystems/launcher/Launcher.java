package frc.robot.subsystems.launcher.java;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public class Launcher extends SubsystemBase {
  private final ColorSensorIO m_sensorIO;
  private final ColorSensorIOInputsAutoLogged m_sensorInputs = new ColorSensorIOInputsAutoLogged();

  public Launcher(){
    // Initialize the ColorSensorIORevV3 object
    m_sensorIO = new ColorSensorIORevV3();
  }

  public boolean noteDetected(){
    // Updated when m_sensorIO.updateInputs(m_sensorInputs) happens in periodic
    return (m_sensorInputs.IsNoteDetected);
  }

  @Override
  public void periodic() {
    // Update the color sensor inputs
    m_sensorIO.updateInputs(m_sensorInputs);
  }
}
